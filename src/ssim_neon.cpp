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

#if !defined(__ARM_NEON) && RMGR_COMPILER_IS_MSVC_AT_LEAST(17,0,0) && RMGR_ARCH_IS_ARM_32
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


#if RMGR_SSIM_USE_DOUBLE
    typedef float64x2_t         Vector;
    typedef float32x2_t         LaneVector;
    #define VEC_SIZE            2
    #define VLD1(addr)          vld1q_f64(addr)
    #define VST1(addr, val)     vst1q_f64((addr), (val))
    #define VADD(a,b)           vaddq_f64((a), (b))
    #define VSUB(a,b)           vsubq_f64((a), (b))
    #define VMUL(a,b)           vmulq_f64((a), (b))
    #define VMUL_LANE(a,b,n)    vmulq_laneq_f64((a), (b), (n))
    #define VMLA(a,b)           vfmaq_f64((a), (b), (c))
    #define VMLA_LANE(a,b,c,n)  vmlaq_lane_f64((a), (b), (c), (n))
    #define VDIV(a,b)           vdivq_f64((a), (b))
    #define VDUP(val)           vdupq_n_f64(val)
#else
    typedef float32x4_t         Vector;
    #define VEC_SIZE            4
    #define VLD1(addr)          vld1q_f32(addr)
    #define VST1(addr, val)     vst1q_f32((addr), (val))
    #define VADD(a,b)           vaddq_f32((a), (b))
    #define VSUB(a,b)           vsubq_f32((a), (b))
    #define VMUL(a,b)           vmulq_f32((a), (b))
#if RMGR_ARCH_IS_ARM_64
    typedef float32x4_t         LaneVector;
    #define LANE_COUNT          4
    #define VLD1_LV(addr)       vld1q_f32(addr)
    #define VMLA(a,b,c)         vfmaq_f32((a), (b), (c))
    #define VMUL_LANE(a,b,n)    vmulq_laneq_f32((a), (b), (n))
    #define VMLA_LANE(a,b,c,n)  vfmaq_laneq_f32((a), (b), (c), (n))
#else
    typedef float32x2_t         LaneVector;
    #define LANE_COUNT          2
    #define VLD1_LV(addr)       vld1_f32(addr)
    #define VMLA(a,b,c)         vmlaq_f32((a), (b), (c))
    #define VMUL_LANE(a,b,n)    vmulq_lane_f32((a), (b), (n))
    #define VMLA_LANE(a,b,c,n)  vmlaq_lane_f32((a), (b), (c), (n))
#endif
    #define VDIV(a,b)           vdivq_f32((a), (b))
    #define VDUP(val)           vdupq_n_f32(val)
#endif


namespace rmgr { namespace ssim { namespace neon
{


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

        // SSE loop. Note that we always process in batches of 4 because we know the buffer is large enough.
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

                // 8x unrolled SSE loop
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
                    d0 = VMLA(d0, s0, k);
                    d1 = VMLA(d1, s1, k);
                    d2 = VMLA(d2, s2, k);
                    d3 = VMLA(d3, s3, k);
                    d4 = VMLA(d4, s4, k);
                    d5 = VMLA(d5, s5, k);
                    d6 = VMLA(d6, s6, k);
                    d7 = VMLA(d7, s7, k);
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
    const unsigned kCount = (21 + VEC_SIZE + 1) & ~(VEC_SIZE-  1);
    static RMGR_ALIGNED_VAR(64, const Float, k21[kCount]) =
    {
        Float(0.07076223776394697),
        Float(0.05666197049168457), Float(0.04537135909566032),
        Float(0.02909122564855043), Float(0.02329443247348710), Float(0.01195976041003701),
        Float(0.00957662749024029), Float(0.00766836382523672), Float(0.00393706926284678), Float(0.00129605559384320),
        Float(0.00202135875836257), Float(0.00161857756253439), Float(0.00083100542908720), Float(0.00027356116008581), Float(0.00005774112519786),
        Float(0.00027356116008581), Float(0.00021905065286602), Float(0.00011246435511668), Float(0.00003702247708275), Float(0.00000781441153305), Float(0.00000105756559815)
    };

    // Preload coeffs in registers
    const unsigned lvCount = kCount / LANE_COUNT;
    LaneVector lv[lvCount];
    for (unsigned i=0; i<lvCount; ++i)
        lv[i] = VLD1_LV(k21 + i*LANE_COUNT);

    #define ki(x,y)               (y * (y+1)/2 + x)
    #define VMUL_K(a, kx, ky)     VMUL_LANE((a), lv[ki(kx,ky) / LANE_COUNT], ki(kx,ky) % LANE_COUNT)
    #define VMLA_K(a, b, kx, ky)  VMLA_LANE((a), (b), lv[ki(kx,ky) / LANE_COUNT], ki(kx,ky) % LANE_COUNT)

    const Float* s = srce -  5*srceStride;
    Float*       d = dest - 10*destStride;

    width = (width + (VEC_SIZE*1)) & ~(VEC_SIZE-1); // Round up width to a multiple of vector length

    int32_t yd = height + 2*radius;
    do
    {
        memset(d+10*destStride, 0, rowSize);
        int32_t xd = width;
        do
        {
            // Unrolled code that exploits all the symmetries in the kernel.
            // Vertical and horizontal symmetries allow us to save a lot of computation thanks to factorization.
            // The diagonal symmetries say that k(x,y) == k(y,x), we cannot exploit that for factorizing,
            // but we can load fewer coefficients by keeping x <= y (21 coefficients instead of 36).

            const Vector s0 = VLD1(s);
            const Vector s1 = VADD(VLD1(s+1), VLD1(s-1)); // Add prior to multiplying due to the vertical line of symmetry
            const Vector s2 = VADD(VLD1(s+2), VLD1(s-2)); // Add prior to multiplying due to the vertical line of symmetry
            const Vector s3 = VADD(VLD1(s+3), VLD1(s-3)); // Add prior to multiplying due to the vertical line of symmetry
            const Vector s4 = VADD(VLD1(s+4), VLD1(s-4)); // Add prior to multiplying due to the vertical line of symmetry
            const Vector s5 = VADD(VLD1(s+5), VLD1(s-5)); // Add prior to multiplying due to the vertical line of symmetry

            Vector sum0 = VMUL_K(s0, 0,0);
            Vector sum1 = VMUL_K(s0, 0,1);
            Vector sum2 = VMUL_K(s0, 0,2);
            Vector sum3 = VMUL_K(s0, 0,3);
            Vector sum4 = VMUL_K(s0, 0,4);
            Vector sum5 = VMUL_K(s0, 0,5);

            sum0 = VMLA_K(sum0, s1, 0,1);
            sum1 = VMLA_K(sum1, s1, 1,1);
            sum2 = VMLA_K(sum2, s1, 1,2);
            sum3 = VMLA_K(sum3, s1, 1,3);
            sum4 = VMLA_K(sum4, s1, 1,4);
            sum5 = VMLA_K(sum5, s1, 1,5);

            sum0 = VMLA_K(sum0, s2, 0,2);
            sum1 = VMLA_K(sum1, s2, 1,2);
            sum2 = VMLA_K(sum2, s2, 2,2);
            sum3 = VMLA_K(sum3, s2, 2,3);
            sum4 = VMLA_K(sum4, s2, 2,4);
            sum5 = VMLA_K(sum5, s2, 2,5);

            sum0 = VMLA_K(sum0, s3, 0,3);
            sum1 = VMLA_K(sum1, s3, 1,3);
            sum2 = VMLA_K(sum2, s3, 2,3);
            sum3 = VMLA_K(sum3, s3, 3,3);
            sum4 = VMLA_K(sum4, s3, 3,4);
            sum5 = VMLA_K(sum5, s3, 3,5);

            sum0 = VMLA_K(sum0, s4, 0,4);
            sum1 = VMLA_K(sum1, s4, 1,4);
            sum2 = VMLA_K(sum2, s4, 2,4);
            sum3 = VMLA_K(sum3, s4, 3,4);
            sum4 = VMLA_K(sum4, s4, 4,4);
            sum5 = VMLA_K(sum5, s4, 4,5);

            sum0 = VMLA_K(sum0, s5, 0,5);
            sum1 = VMLA_K(sum1, s5, 1,5);
            sum2 = VMLA_K(sum2, s5, 2,5);
            sum3 = VMLA_K(sum3, s5, 3,5);
            sum4 = VMLA_K(sum4, s5, 4,5);
            sum5 = VMLA_K(sum5, s5, 5,5);

            // Reuse sums thanks to horizontal line of symmetry
            Vector* vd = reinterpret_cast<Vector*>(d);
            *vd = VADD(sum5, *vd);  vd+=destStride/VEC_SIZE;
            *vd = VADD(sum4, *vd);  vd+=destStride/VEC_SIZE;
            *vd = VADD(sum3, *vd);  vd+=destStride/VEC_SIZE;
            *vd = VADD(sum2, *vd);  vd+=destStride/VEC_SIZE;
            *vd = VADD(sum1, *vd);  vd+=destStride/VEC_SIZE;
            *vd = VADD(sum0, *vd);  vd+=destStride/VEC_SIZE;
            *vd = VADD(sum1, *vd);  vd+=destStride/VEC_SIZE;
            *vd = VADD(sum2, *vd);  vd+=destStride/VEC_SIZE;
            *vd = VADD(sum3, *vd);  vd+=destStride/VEC_SIZE;
            *vd = VADD(sum4, *vd);  vd+=destStride/VEC_SIZE;
            *vd = VADD(sum5, *vd);

            s += VEC_SIZE;
            d += VEC_SIZE;
        }
        while ((xd -= VEC_SIZE) != 0);

        s += srceStride - width;
        d += destStride - width;
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
