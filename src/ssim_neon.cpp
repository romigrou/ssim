/*
 * Copyright (c) 2020, Romain Bailly
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "ssim_internal.h"

#if !defined(__ARM_NEON) && ((RMGR_COMPILER_IS_MSVC_AT_LEAST(17,0,0) && RMGR_ARCH_IS_ARM_32) || (RMGR_COMPILER_IS_MSVC_AT_LEAST(19,20,0) && RMGR_ARCH_IS_ARM_64))
    #define __ARM_NEON  1
#endif

#if !defined(__ARM_NEON) || (RMGR_SSIM_USE_DOUBLE && RMGR_ARCH_IS_ARM_32)

namespace rmgr { namespace ssim { namespace neon
{
    const MultiplyFct     g_multiplyFct     = NULL;
    const GaussianBlurFct g_gaussianBlurFct = NULL;
    const SumTileFct      g_sumTileFct      = NULL;
}}}

#else

#include <arm_neon.h>
#include <cassert>
#include <cstring>

#define EXPAND(x)  x

#if RMGR_SSIM_USE_DOUBLE
    typedef float64x2_t             Vector;
    #define VEC_SIZE                2
    #define VLD1(addr)              vld1q_f64(addr)
    #define VST1(addr, val)         vst1q_f64((addr), (val))
    #define VADD(a,b)               vaddq_f64((a), (b))
    #define VSUB(a,b)               vsubq_f64((a), (b))
    #define VMUL(a,b)               vmulq_f64((a), (b))
    #define VDIV(a,b)               vdivq_f64((a), (b))
    #define VDUP(val)               vdupq_n_f64(val)
    typedef float64x2_t             KVector;
    #define KVEC_SIZE               2
    #define KVLD1(addr)             vld1q_f64(addr)
    #define VMLA(acc,a,b)           acc = vfmaq_f64(acc, a, b)
    #define VMUL_LANE(res,...)      res = EXPAND(vmulq_laneq_f64(__VA_ARGS__))
    #define VMLA_LANE(acc,...)      acc = EXPAND(vmlaq_laneq_f64(acc, __VA_ARGS__))

#else

    typedef float32x4_t             Vector;
    #define VEC_SIZE                4
    #define VLD1(addr)              vld1q_f32(addr)
    #define VST1(addr, val)         vst1q_f32((addr), (val))
    #define VADD(a,b)               vaddq_f32((a), (b))
    #define VSUB(a,b)               vsubq_f32((a), (b))
    #define VMUL(a,b)               vmulq_f32((a), (b))
    #define VDIV(a,b)               vdivq_f32((a), (b))
    #define VDUP(val)               vdupq_n_f32(val)
#if RMGR_ARCH_IS_ARM_64 && !RMGR_COMPILER_IS_MSVC
    typedef float32x4_t             KVector;
    #define KVEC_SIZE               4
    #define KVLD1(addr)             vld1q_f32(addr)
    #define VMLA(acc,a,b)           acc = vfmaq_f32(acc, a, b)
    #define VMUL_LANE(res,...)      res = EXPAND(vmulq_laneq_f32(__VA_ARGS__))
    #define VMLA_LANE(acc,...)      acc = EXPAND(vmlaq_laneq_f32(acc, __VA_ARGS__))
#else
    typedef float32x2_t             KVector;
    #define KVEC_SIZE               2
    #define KVLD1(addr)             vld1_f32(addr)
    #define VMLA(acc,a,b)           acc = vmlaq_f32(acc, a, b)
    #define VMUL_LANE(res,...)      res = EXPAND(vmulq_lane_f32(__VA_ARGS__))
    #define VMLA_LANE(acc,...)      acc = EXPAND(vmlaq_lane_f32(acc, __VA_ARGS__))
#endif
#endif


namespace rmgr { namespace ssim { namespace neon
{


// Clang doesn't seem to be willing to issue the multiply vector by scalar instructions,
// even though we specify the corresponding intrinsics. Let's force it to do so.
// Note: haven't tried with GCC...
#if RMGR_COMPILER_IS_CLANG
 #if RMGR_ARCH_IS_ARM_64 // Somehow, using the intrinsic for vmul yields better code in 32-bit mode
    #undef VMUL_LANE
    #define VMUL_LANE(res,...)  VMUL_LANE_ASM(res, __VA_ARGS__)
#endif
    #undef VMLA_LANE
    #define VMLA_LANE(acc,...)  VMLA_LANE_ASM(acc, __VA_ARGS__)
    #if RMGR_ARCH_IS_ARM_32
        #define REGISTER(r32, r64, type, name)  register type name __asm__(#r32)
        #define VMUL_LANE_ASM(res,v1,v2,n)  __asm__("vmul.f32 %0, %1, %2[" #n "]": "+w"(res): "w"(v1), "w"(v2))
        #define VMLA_LANE_ASM(acc,v1,v2,n)  __asm__("vmla.f32 %0, %1, %2[" #n "]": "+w"(acc): "w"(v1), "w"(v2))
    #elif RMGR_ARCH_IS_ARM_64 && !RMGR_SSIM_USE_DOUBLE
        #define REGISTER(r32, r64, type, name)  register type name __asm__(#r64)
        #define VMUL_LANE_ASM(res,v1,v2,n)  __asm__("fmul %0.4s, %1.4s, %2.s[" #n "]": "+w"(res): "w"(v1), "w"(v2))
        #define VMLA_LANE_ASM(acc,v1,v2,n)  __asm__("fmla %0.4s, %1.4s, %2.s[" #n "]": "+w"(acc): "w"(v1), "w"(v2))
    #else
        #define REGISTER(r32, r64, type, name)  register type name __asm__(#r64)
        #define VMUL_LANE_ASM(res,v1,v2,n)  __asm__("fmul %0.2d, %1.2d, %2.d[" #n "]": "+w"(res): "w"(v1), "w"(v2))
        #define VMLA_LANE_ASM(acc,v1,v2,n)  __asm__("fmla %0.2d, %1.2d, %2.d[" #n "]": "+w"(acc): "w"(v1), "w"(v2))
    #endif
#else
    #define REGISTER(r32, r64, type, name)  type name
#endif


//=================================================================================================
// multiply()

static void multiply(Float* product, const Float* a, const Float* b, uint32_t width, uint32_t height, size_t stride, uint32_t margin) RMGR_NOEXCEPT
{

    a       -= margin * stride + margin;
    b       -= margin * stride + margin;
    product -= margin * stride + margin;
    for (uint32_t y=0; y<height+2*margin; ++y)
    {
        // Process left margin as scalar so as to deal with aligned pointers in the rest
        for (uint32_t x=0; x<margin; ++x)
            product[x] = a[x] * b[x];

        // Neon loop. Note that we always process in batches of 4 because we know the buffer is large enough.
        const Vector* va = reinterpret_cast<const Vector*>(a + margin);
        const Vector* vb = reinterpret_cast<const Vector*>(b + margin);
        Vector*       vp = reinterpret_cast<Vector*>(product + margin);
        int32_t x = width + margin;
        do
        {
            *vp++ = VMUL(*va++, *vb++);
            *vp++ = VMUL(*va++, *vb++);
            *vp++ = VMUL(*va++, *vb++);
            *vp++ = VMUL(*va++, *vb++);
            *vp++ = VMUL(*va++, *vb++);
            *vp++ = VMUL(*va++, *vb++);
            *vp++ = VMUL(*va++, *vb++);
            *vp++ = VMUL(*va++, *vb++);
        }
        while ((x -= 8*VEC_SIZE) >= 0);

        a       += stride;
        b       += stride;
        product += stride;
    }
}


const MultiplyFct g_multiplyFct = multiply;


//=================================================================================================
// gaussian_blur()

static void gaussian_blur(Float* dest, ptrdiff_t destStride, const Float* srce, ptrdiff_t srceStride, int32_t width, int32_t height, const Float kernel[], int radius) RMGR_NOEXCEPT
{
    assert(width  > 0);
    assert(height > 0);
    assert(srceStride >= width + 2*radius);
    assert(srceStride % VEC_SIZE == 0);
    assert(destStride % VEC_SIZE == 0);
    assert(reinterpret_cast<uintptr_t>(srce) % 16 == 0);
    assert(reinterpret_cast<uintptr_t>(dest) % 16 == 0);

    const size_t rowSize = width * sizeof(Float);

#if 0
    // Generic Neon implementation.
    // Note that we always process in batches of 8 because we know the buffer is large enough.

    const int32_t kernelStride = 2*radius + 1;

    for (int32_t yd=0; yd<height; ++yd)
    {
        memset(dest, 0, rowSize);
        for (int32_t yk=-radius; yk<=radius; ++yk)
        {
            const Float* kernelRow = kernel + yk * kernelStride;
            for (int32_t xk=-radius; xk<=radius; ++xk)
            {
                const Vector k = VDUP(kernelRow[xk]);
                const Float* s = srce + (yd+yk) * srceStride + xk;
                Float*       d = dest;

                int32_t xd = width;

                // 8x unrolled Neon loop
                while ((xd -= 8*VEC_SIZE) >= 0)
                {
                    Vector d0 = VLD1(d);
                    Vector s0 = VLD1(s);
                    Vector d1 = VLD1(d + VEC_SIZE);
                    Vector s1 = VLD1(s + VEC_SIZE);
                    Vector d2 = VLD1(d + VEC_SIZE*2);
                    Vector s2 = VLD1(s + VEC_SIZE*2);
                    Vector d3 = VLD1(d + VEC_SIZE*3);
                    Vector s3 = VLD1(s + VEC_SIZE*3);
                    Vector d4 = VLD1(d + VEC_SIZE*4);
                    Vector s4 = VLD1(s + VEC_SIZE*4);
                    Vector d5 = VLD1(d + VEC_SIZE*5);
                    Vector s5 = VLD1(s + VEC_SIZE*5);
                    Vector d6 = VLD1(d + VEC_SIZE*6);
                    Vector s6 = VLD1(s + VEC_SIZE*6);
                    Vector d7 = VLD1(d + VEC_SIZE*7);
                    Vector s7 = VLD1(s + VEC_SIZE*7);
                    VMLA(d0, s0, k);
                    VMLA(d1, s1, k);
                    VMLA(d2, s2, k);
                    VMLA(d3, s3, k);
                    VMLA(d4, s4, k);
                    VMLA(d5, s5, k);
                    VMLA(d6, s6, k);
                    VMLA(d7, s7, k);
                    VST1(d + VEC_SIZE*0, d0);
                    VST1(d + VEC_SIZE*1, d1);
                    VST1(d + VEC_SIZE*2, d2);
                    VST1(d + VEC_SIZE*3, d3);
                    VST1(d + VEC_SIZE*4, d4);
                    VST1(d + VEC_SIZE*5, d5);
                    VST1(d + VEC_SIZE*6, d6);
                    VST1(d + VEC_SIZE*7, d7);
                    s += 8 * VEC_SIZE;
                    d += 8 * VEC_SIZE;
                }
                xd += 8 * VEC_SIZE;
            }
        }

        dest += destStride;
    }

#else

    // Faster version, but tailored for the radius==5 case
    // Here too we don't care about buffer overrun as we know the buffer was allocated accordingly.

    assert(radius == 5);
    (void)kernel;

    // The precomputed 21 unique values of the Gaussian kernel
    const unsigned kCount = (21 + KVEC_SIZE + 1) & ~(KVEC_SIZE-  1);
    static RMGR_ALIGNED_VAR(64, const Float, k21[kCount]) =
    {
        Float(7.07622393965721130e-02),
        Float(5.66619709134101868e-02), Float(4.53713610768318176e-02),
        Float(2.90912277996540070e-02), Float(2.32944320887327194e-02), Float(1.19597595185041428e-02),
        Float(9.57662798464298248e-03), Float(7.66836293041706085e-03), Float(3.93706932663917542e-03), Float(1.29605561960488558e-03),
        Float(2.02135881409049034e-03), Float(1.61857774946838617e-03), Float(8.31005279906094074e-04), Float(2.73561221547424793e-04), Float(5.77411265112459660e-05),
        Float(2.73561221547424793e-04), Float(2.19050692976452410e-04), Float(1.12464345875196159e-04), Float(3.70224843209143728e-05), Float(7.81441485742107034e-06), Float(1.05756600987660931e-06)
    };

    // Preload coeffs in registers
    REGISTER(d0, v0, const KVector, k0)  = KVLD1(k21);
    REGISTER(d1, v1, const KVector, k1)  = KVLD1(k21 + KVEC_SIZE);
    REGISTER(d2, v2, const KVector, k2)  = KVLD1(k21 + KVEC_SIZE*2);
    REGISTER(d3, v3, const KVector, k3)  = KVLD1(k21 + KVEC_SIZE*3);
    REGISTER(d4, v4, const KVector, k4)  = KVLD1(k21 + KVEC_SIZE*4);
    REGISTER(d5, v5, const KVector, k5)  = KVLD1(k21 + KVEC_SIZE*5);
#if KVEC_SIZE == 2
    REGISTER(d6, v6, const KVector, k6)  = KVLD1(k21 + KVEC_SIZE*6);
    REGISTER(d7, v7, const KVector, k7)  = KVLD1(k21 + KVEC_SIZE*7);
    REGISTER(d8, v8, const KVector, k8)  = KVLD1(k21 + KVEC_SIZE*8);
    REGISTER(d9, v9, const KVector, k9)  = KVLD1(k21 + KVEC_SIZE*9);
    REGISTER(d10,v10,const KVector, k10) = KVLD1(k21 + KVEC_SIZE*10);
#endif

    // Pre-loaded registers and indexes for each of the kernel coefficients
    #define K00   k0,0
    #define K01   k0,1
#if KVEC_SIZE == 2
    #define K02   k1,1
    #define K03   k3,0
    #define K04   k5,0
    #define K05   k7,1
    #define K11   k1,0
    #define K12   k2,0
    #define K13   k3,1
    #define K14   k5,1
    #define K15   k8,0
    #define K22   k2,1
    #define K23   k4,0
    #define K24   k6,0
    #define K25   k8,1
    #define K33   k4,1
    #define K34   k6,1
    #define K35   k9,0
    #define K44   k7,0
    #define K45   k9,1
    #define K55   k10,0
#else
    #define K02   k0,3
    #define K03   k1,2
    #define K04   k2,2
    #define K05   k3,3
    #define K11   k0,2
    #define K12   k1,0
    #define K13   k1,3
    #define K14   k2,3
    #define K15   k4,0
    #define K22   k1,1
    #define K23   k2,0
    #define K24   k3,0
    #define K25   k4,1
    #define K33   k2,1
    #define K34   k3,1
    #define K35   k4,2
    #define K44   k3,2
    #define K45   k4,3
    #define K55   k5,0
#endif

    #define VMUL_K(res, v, k)  EXPAND_AND_CALL(VMUL_LANE, res, v, k)
    #define VMLA_K(res, v, k)  EXPAND_AND_CALL(VMLA_LANE, res, v, k)

    const Float* s = srce -  5*srceStride;
    Float*       d = dest - 10*destStride;

    width = (width + (VEC_SIZE*1)) & ~(VEC_SIZE-1); // Round up width to a multiple of vector length

    int32_t yd = height + 2*radius;
    do
    {
        memset(d+10*destStride, 0, rowSize);
        int32_t xd = width;
        const ptrdiff_t vdStride = destStride / VEC_SIZE;
        Vector* vd_5 = reinterpret_cast<Vector*>(d);
        Vector* vd_3 = vd_5 + 2*vdStride;
        Vector* vd_1 = vd_3 + 2*vdStride;
        Vector* vd1  = vd_1 + 2*vdStride;
        Vector* vd3  = vd1  + 2*vdStride;
        Vector* vd5  = vd3  + 2*vdStride;

        do
        {
            // Unrolled code that exploits all the symmetries in the kernel.
            // Vertical and horizontal symmetries allow us to save a lot of computation thanks to factorization.
            // The diagonal symmetries say that k(x,y) == k(y,x), we cannot exploit that for factorizing,
            // but we can load fewer coefficients by keeping x <= y (21 coefficients instead of 36).

            Vector sum0, sum1, sum2, sum3, sum4, sum5;

            const Vector s1  = VLD1(s+1);
            const Vector s_1 = VLD1(s-1);
            const Vector s0  = VLD1(s);

            const Vector s11 = VADD(s1, s_1); // Add prior to multiplying due to the vertical line of symmetry
            const Vector s2  = VLD1(s+2);
            const Vector s_2 = VLD1(s-2);
            VMUL_LANE(sum0, s0, K00);
            VMUL_LANE(sum0, s0, K00);
            VMUL_LANE(sum1, s0, K01);
            VMUL_LANE(sum2, s0, K02);
            VMUL_LANE(sum3, s0, K03);
            VMUL_LANE(sum4, s0, K04);
            VMUL_LANE(sum5, s0, K05);

            const Vector s22 = VADD(s2, s_2); // Add prior to multiplying due to the vertical line of symmetry
            const Vector s3  = VLD1(s+3);
            const Vector s_3 = VLD1(s-3);
            VMLA_LANE(sum0, s11, K01);
            VMLA_LANE(sum1, s11, K11);
            VMLA_LANE(sum2, s11, K12);
            VMLA_LANE(sum3, s11, K13);
            VMLA_LANE(sum4, s11, K14);
            VMLA_LANE(sum5, s11, K15);

            const Vector s33 = VADD(s3, s_3); // Add prior to multiplying due to the vertical line of symmetry
            const Vector s4  = VLD1(s+4);
            const Vector s_4 = VLD1(s-4);
            VMLA_LANE(sum0, s22, K02);
            VMLA_LANE(sum1, s22, K12);
            VMLA_LANE(sum2, s22, K22);
            VMLA_LANE(sum3, s22, K23);
            VMLA_LANE(sum4, s22, K24);
            VMLA_LANE(sum5, s22, K25);

            const Vector s44 = VADD(s4, s_4); // Add prior to multiplying due to the vertical line of symmetry
            const Vector s5  = VLD1(s+5);
            const Vector s_5 = VLD1(s-5);
            VMLA_LANE(sum0, s33, K03);
            VMLA_LANE(sum1, s33, K13);
            VMLA_LANE(sum2, s33, K23);
            VMLA_LANE(sum3, s33, K33);
            VMLA_LANE(sum4, s33, K34);
            VMLA_LANE(sum5, s33, K35);

            const Vector s55 = VADD(s5, s_5); // Add prior to multiplying due to the vertical line of symmetry
            VMLA_LANE(sum0, s44, K04);
            VMLA_LANE(sum1, s44, K14);
            VMLA_LANE(sum2, s44, K24);
            VMLA_LANE(sum3, s44, K34);
            VMLA_LANE(sum4, s44, K44);
            VMLA_LANE(sum5, s44, K45);

            VMLA_LANE(sum0, s55, K05);
            VMLA_LANE(sum1, s55, K15);
            VMLA_LANE(sum2, s55, K25);
            VMLA_LANE(sum3, s55, K35);
            VMLA_LANE(sum4, s55, K45);
            VMLA_LANE(sum5, s55, K55);

            // Reuse sums thanks to horizontal line of symmetry
            Vector d0  = vd_1[vdStride];
            Vector d_1 = vd_1[0];
            Vector d1  = vd1[0];
            Vector d_2 = vd_3[vdStride];
            Vector d2  = vd1[vdStride];
            Vector d_3 = vd_3[0];
            Vector d3  = vd3[0];
            Vector d_4 = vd_5[vdStride];
            Vector d4  = vd3[vdStride];
            Vector d_5 = vd_5[0];
            Vector d5  = vd5[0];
            d0  = VADD(d0,  sum0);
            d_1 = VADD(d_1, sum1);
            d1  = VADD(d1,  sum1);
            d_2 = VADD(d_2, sum2);
            d2  = VADD(d2,  sum2);
            d_3 = VADD(d_3, sum3);
            d3  = VADD(d3,  sum3);
            d_4 = VADD(d_4, sum4);
            d4  = VADD(d4,  sum4);
            d_5 = VADD(d_5, sum5);
            d5  = VADD(d5,  sum5);
            vd_1[vdStride] = d0;
            vd_1[0]        = d_1;
            vd1[0]         = d1;
            vd_3[vdStride] = d_2;
            vd1[vdStride]  = d2;
            vd_3[0]        = d_3;
            vd3[0]         = d3;
            vd_5[vdStride] = d_4;
            vd3[vdStride]  = d4;
            vd_5[0]        = d_5;
            vd5[0]         = d5;

            ++vd_5;
            ++vd_3;
            ++vd_1;
            ++vd1;
            ++vd3;
            ++vd5;

            s += VEC_SIZE;
        }
        while ((xd -= VEC_SIZE) != 0);

        s += srceStride - width;
        d += destStride;
    }
    while (--yd);

    #undef k

#endif
}


const GaussianBlurFct g_gaussianBlurFct = gaussian_blur;


//=================================================================================================
// sum_tile()


const SumTileFct g_sumTileFct = NULL;


}}} // namespace rmgr::ssim::neon

#endif
