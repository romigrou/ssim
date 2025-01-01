/*
 * Copyright (c) 2023, Romain Bailly
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

#if RMGR_ARCH_IS_X86_ANY

#if !defined(__FMA__) && RMGR_COMPILER_IS_MSVC_AT_LEAST(18,0,0)
    #define __FMA__  1
#endif

#ifndef RMGR_SSIM_USE_128BIT_FMA
    #define  RMGR_SSIM_USE_128BIT_FMA  0
#endif

#if !defined(__FMA__)

namespace rmgr { namespace ssim { namespace fma
{
    const GaussianBlurFct g_gaussianBlurFct = NULL;
}}}

#else

#include <cassert>
#include <cstring>
#include <immintrin.h>

namespace rmgr { namespace ssim { namespace fma
{

#if RMGR_SSIM_USE_128BIT_FMA
    // 128-bit FMA
    #if RMGR_SSIM_USE_DOUBLE
        typedef __m128d        Vector;
        #define VEC_SIZE       2
        #define VCOEFF(val)    val, val
        #define VLOADA(addr)   _mm_load_pd((addr))
        #define VLOADU(addr)   _mm_loadu_pd((addr))
        #define VADD(a,b)      _mm_add_pd((a), (b))
        #define VSUB(a,b)      _mm_sub_pd((a), (b))
        #define VMUL(a,b)      _mm_mul_pd((a), (b))
        #define VFMADD(a,b,c)  _mm_fmadd_pd((a), (b), (c))
        #define VSET1(val)     _mm_set1_ps(val)
    #else
        typedef __m128         Vector;
        #define VEC_SIZE       4
        #define VCOEFF(val)    val##f, val##f, val##f, val##f
        #define VLOADA(addr)   _mm_load_ps((addr))
        #define VLOADU(addr)   _mm_loadu_ps((addr))
        #define VADD(a,b)      _mm_add_ps((a), (b))
        #define VSUB(a,b)      _mm_sub_ps((a), (b))
        #define VMUL(a,b)      _mm_mul_ps((a), (b))
        #define VFMADD(a,b,c)  _mm_fmadd_ps((a), (b), (c))
        #define VSET1(val)     _mm_set1_ps(val)
    #endif

#else

    // 256-bit FMA
    #if RMGR_SSIM_USE_DOUBLE
        typedef __m256d        Vector;
        #define VEC_SIZE       4
        #define VCOEFF(val)    val, val, val, val
        #define VLOADA(addr)   _mm256_load_pd((addr))
        #define VLOADU(addr)   _mm256_loadu_pd((addr))
        #define VADD(a,b)      _mm256_add_pd((a), (b))
        #define VSUB(a,b)      _mm256_sub_pd((a), (b))
        #define VMUL(a,b)      _mm256_mul_pd((a), (b))
        #define VFMADD(a,b,c)  _mm256_fmadd_pd((a), (b), (c))
        #define VSET1(val)     _mm256_set1_pd(val)
    #else
        typedef __m256         Vector;
        #define VEC_SIZE       8
        #define VCOEFF(val)    val##f, val##f, val##f, val##f, val##f, val##f, val##f, val##f
        #define VLOADA(addr)   _mm256_load_ps((addr))
        #define VLOADU(addr)   _mm256_loadu_ps((addr))
        #define VADD(a,b)      _mm256_add_ps((a), (b))
        #define VSUB(a,b)      _mm256_sub_ps((a), (b))
        #define VMUL(a,b)      _mm256_mul_ps((a), (b))
        #define VFMADD(a,b,c)  _mm256_fmadd_ps((a), (b), (c))
        #define VSET1(val)     _mm256_set1_ps(val)
    #endif
#endif

#if RMGR_SSIM_USE_DOUBLE
    typedef __m128d Scalar;
    #define SET     _mm_set_sd
    #define LOAD    _mm_load_sd
    #define STORE   _mm_store_sd
    #define MUL     _mm_mul_sd
    #define FMADD   _mm_fmadd_sd
#else
    typedef __m128  Scalar;
    #define SET     _mm_set_ss
    #define LOAD    _mm_load_ss
    #define STORE   _mm_store_ss
    #define MUL     _mm_mul_ss
    #define FMADD   _mm_fmadd_ss
#endif


//=================================================================================================
// gaussian_pass()

static void gaussian_blur_pass(Float* dest, ptrdiff_t destStride, const Float* srce, ptrdiff_t srceStride, int32_t width, int32_t height, const Float /*kernel*/[]) RMGR_NOEXCEPT
{
    assert(reinterpret_cast<uintptr_t>(dest) % 16 == 0); // dest must be aligned on 16 bytes
    assert(destStride % (16/sizeof(Float)) == 0);        // destStride be also be a multiple of 16 bytes

    const Float k5 = Float(1.028380084479109868e-03);
    const Float k4 = Float(7.598758135239185030e-03);
    const Float k3 = Float(3.600077212843082186e-02);
    const Float k2 = Float(1.093606895097000153e-01);
    const Float k1 = Float(2.130055377112536896e-01);
    const Float k0 = Float(2.660117248617943631e-01);

    const Vector vk5 = VSET1(k5);
    const Vector vk4 = VSET1(k4);
    const Vector vk3 = VSET1(k3);
    const Vector vk2 = VSET1(k2);
    const Vector vk1 = VSET1(k1);
    const Vector vk0 = VSET1(k0);

    const Scalar sk5 = SET(k5);
    const Scalar sk4 = SET(k4);
    const Scalar sk3 = SET(k3);
    const Scalar sk2 = SET(k2);
    const Scalar sk1 = SET(k1);
    const Scalar sk0 = SET(k0);

    int32_t y = 0;
    const int32_t yUnrolling = 4;
    for (; y+yUnrolling <= height; y+=yUnrolling)
    {
        const Float* s0 = srce;
        const Float* s1 = srce + srceStride;
        const Float* s2 = srce + srceStride*2;
        const Float* s3 = srce + srceStride*3;

        // Main SIMD loop
        // We always process 4 rows at a time, which for floats means in blocks of 8x4 because 8 rows
        // would require too many registers (even worse in 32-bit x86 as there are only 8 AVX registers).
        // The double implementation processes 4x4 blocks and, therefore, looks a lot like the SSE one.
        ptrdiff_t x = 0;
        for (; x+VEC_SIZE <= width; x+=VEC_SIZE)
        {
            #define D_5(yi)   Vector d##yi = VMUL(VLOADU(s##yi + x - 5), vk5);       \
                              d##yi = VFMADD(VLOADU(s##yi + x + 5),  vk5,    d##yi)
            #define D(xi,yi)  d##yi = VFMADD(VLOADU(s##yi + x - xi), vk##xi, d##yi); \
                              d##yi = VFMADD(VLOADU(s##yi + x + xi), vk##xi, d##yi)
            #define D_0(yi)   d##yi = VFMADD(VLOADU(s##yi+x), vk0, d##yi)

            // Compute Gaussian blur
            D_5(0); D_5(1);  D_5(2);  D_5(3);
            D(4,0);  D(4,1);  D(4,2);  D(4,3);
            D(3,0);  D(3,1);  D(3,2);  D(3,3);
            D(2,0);  D(2,1);  D(2,2);  D(2,3);
            D(1,0);  D(1,1);  D(1,2);  D(1,3);
            D_0(0);  D_0(1);  D_0(2);  D_0(3);

            // Tranpose and write
#if RMGR_SSIM_USE_DOUBLE
            const __m256d tmp0 = _mm256_shuffle_pd(d0, d1,  0);
            const __m256d tmp1 = _mm256_shuffle_pd(d0, d1, 15);
            const __m256d tmp2 = _mm256_shuffle_pd(d2, d3,  0);
            const __m256d tmp3 = _mm256_shuffle_pd(d2, d3, 15);

            _mm_store_pd(dest,   _mm256_castpd256_pd128(tmp0));
            _mm_store_pd(dest+2, _mm256_castpd256_pd128(tmp2));   dest+=destStride;
            _mm_store_pd(dest,   _mm256_castpd256_pd128(tmp1));
            _mm_store_pd(dest+2, _mm256_castpd256_pd128(tmp3));   dest+=destStride;
            _mm_store_pd(dest,   _mm256_extractf128_pd(tmp0,1));
            _mm_store_pd(dest+2, _mm256_extractf128_pd(tmp2,1));  dest+=destStride;
            _mm_store_pd(dest,   _mm256_extractf128_pd(tmp1,1));
            _mm_store_pd(dest+2, _mm256_extractf128_pd(tmp3,1));  dest+=destStride;
#else
            const __m256 tmp0 = _mm256_shuffle_ps(d0, d1, 0x44);
            const __m256 tmp2 = _mm256_shuffle_ps(d0, d1, 0xEE);
            const __m256 tmp1 = _mm256_shuffle_ps(d2, d3, 0x44);
            const __m256 tmp3 = _mm256_shuffle_ps(d2, d3, 0xEE);
            d0 = _mm256_shuffle_ps(tmp0, tmp1, 0x88);
            d1 = _mm256_shuffle_ps(tmp0, tmp1, 0xDD);
            d2 = _mm256_shuffle_ps(tmp2, tmp3, 0x88);
            d3 = _mm256_shuffle_ps(tmp2, tmp3, 0xDD);

            _mm_store_ps(dest, _mm256_castps256_ps128(d0));   dest+=destStride;
            _mm_store_ps(dest, _mm256_castps256_ps128(d1));   dest+=destStride;
            _mm_store_ps(dest, _mm256_castps256_ps128(d2));   dest+=destStride;
            _mm_store_ps(dest, _mm256_castps256_ps128(d3));   dest+=destStride;
            _mm_store_ps(dest, _mm256_extractf128_ps(d0,1));  dest+=destStride;
            _mm_store_ps(dest, _mm256_extractf128_ps(d1,1));  dest+=destStride;
            _mm_store_ps(dest, _mm256_extractf128_ps(d2,1));  dest+=destStride;
            _mm_store_ps(dest, _mm256_extractf128_ps(d3,1));  dest+=destStride;
#endif

            #undef D_5
            #undef D
            #undef D_0
        }

        // Scalar epilogue for width
        for (; x < width; ++x)
        {
            #define D_5(yi)   Scalar d##yi = MUL(LOAD(s##yi + x - 5), sk5);       \
                              d##yi = FMADD(LOAD(s##yi + x + 5),  sk5,    d##yi)
            #define D(xi,yi)  d##yi = FMADD(LOAD(s##yi + x - xi), sk##xi, d##yi); \
                              d##yi = FMADD(LOAD(s##yi + x + xi), sk##xi, d##yi)
            #define D_0(yi)   d##yi = FMADD(LOAD(s##yi+x), sk0, d##yi)

            D_5(0);  D_5(1);  D_5(2);  D_5(3);
            D(4,0);  D(4,1);  D(4,2);  D(4,3);
            D(3,0);  D(3,1);  D(3,2);  D(3,3);
            D(2,0);  D(2,1);  D(2,2);  D(2,3);
            D(1,0);  D(1,1);  D(1,2);  D(1,3);
            D_0(0);  D_0(1);  D_0(2);  D_0(3);

            STORE(dest+0, d0);
            STORE(dest+1, d1);
            STORE(dest+2, d2);
            STORE(dest+3, d3);
            dest += destStride;

            #undef D_5
            #undef D
            #undef D_0
        }

        srce += srceStride * yUnrolling;
        dest -= (destStride * width) - yUnrolling;
    }

    // Scalar epilogue for height
    for (; y < height; ++y)
    {
        for (ptrdiff_t x=0; x < width; ++x)
        {
            Scalar d;
            d = MUL(  LOAD(srce+x-5), sk5);
            d = FMADD(LOAD(srce+x+5), sk5, d);
            d = FMADD(LOAD(srce+x-4), sk4, d);
            d = FMADD(LOAD(srce+x+4), sk4, d);
            d = FMADD(LOAD(srce+x-3), sk3, d);
            d = FMADD(LOAD(srce+x+3), sk3, d);
            d = FMADD(LOAD(srce+x-2), sk2, d);
            d = FMADD(LOAD(srce+x+2), sk2, d);
            d = FMADD(LOAD(srce+x-1), sk1, d);
            d = FMADD(LOAD(srce+x+1), sk1, d);
            d = FMADD(LOAD(srce+x),   sk0, d);
            STORE(dest, d);
            dest += destStride;
        }

        srce += srceStride;
        dest -= (destStride * width) - 1;
    }

    _mm256_zeroupper();
}


const GaussianPassFct g_gaussianPassFct = gaussian_blur_pass;


//=================================================================================================
// gaussian_blur()

static void gaussian_blur(Float* dest, ptrdiff_t destStride, const Float* srce, ptrdiff_t srceStride, int32_t width, int32_t height, const Float kernel[], int radius) RMGR_NOEXCEPT
{
    assert(width  > 0);
    assert(height > 0);
    assert(srceStride >= width + 2*radius);
    assert(srceStride % VEC_SIZE == 0);
    assert(destStride % VEC_SIZE == 0);
    assert(reinterpret_cast<uintptr_t>(srce) % (VEC_SIZE * sizeof(Float)) == 0);
    assert(reinterpret_cast<uintptr_t>(dest) % (VEC_SIZE * sizeof(Float)) == 0);

    const size_t rowSize = width * sizeof(Float);

#if 0
    // Generic FMA implementation.
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
                const Vector k  = VSET1(kernelRow[xk]);
                const Float* s  = srce + (yd+yk) * srceStride + xk;
                Vector*      vd = reinterpret_cast<Vector*>(dest);

                int32_t xd = width;

                // 8x unrolled FMA loop
                while ((xd -= 8*VEC_SIZE) >= 0)
                {
                    vd[0] = VFMADD(VLOADU(s), k, vd[0]);  s+=VEC_SIZE;
                    vd[1] = VFMADD(VLOADU(s), k, vd[1]);  s+=VEC_SIZE;
                    vd[2] = VFMADD(VLOADU(s), k, vd[2]);  s+=VEC_SIZE;
                    vd[3] = VFMADD(VLOADU(s), k, vd[3]);  s+=VEC_SIZE;
                    vd[4] = VFMADD(VLOADU(s), k, vd[4]);  s+=VEC_SIZE;
                    vd[5] = VFMADD(VLOADU(s), k, vd[5]);  s+=VEC_SIZE;
                    vd[6] = VFMADD(VLOADU(s), k, vd[6]);  s+=VEC_SIZE;
                    vd[7] = VFMADD(VLOADU(s), k, vd[7]);  s+=VEC_SIZE;
                    vd += 8;
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
    static RMGR_ALIGNED_VAR(64, const Float, k21[21 * VEC_SIZE]) =
    {
        VCOEFF(7.07622393965721130e-02),
        VCOEFF(5.66619709134101868e-02), VCOEFF(4.53713610768318176e-02),
        VCOEFF(2.90912277996540070e-02), VCOEFF(2.32944320887327194e-02), VCOEFF(1.19597595185041428e-02),
        VCOEFF(9.57662798464298248e-03), VCOEFF(7.66836293041706085e-03), VCOEFF(3.93706932663917542e-03), VCOEFF(1.29605561960488558e-03),
        VCOEFF(2.02135881409049034e-03), VCOEFF(1.61857774946838617e-03), VCOEFF(8.31005279906094074e-04), VCOEFF(2.73561221547424793e-04), VCOEFF(5.77411265112459660e-05),
        VCOEFF(2.73561221547424793e-04), VCOEFF(2.19050692976452410e-04), VCOEFF(1.12464345875196159e-04), VCOEFF(3.70224843209143728e-05), VCOEFF(7.81441485742107034e-06), VCOEFF(1.05756600987660931e-06)
    };

    const Float* s = srce -  5*srceStride;
    Float*       d = dest - 10*destStride;

    #define k(x,y)  *reinterpret_cast<const Vector*>(&k21[((y) * ((y)+1)/2 + (x)) * VEC_SIZE])

    width = (width + (2*VEC_SIZE*1)) & ~(2*VEC_SIZE-1); // Round up width to a multiple of vector length

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

            const Vector s0 = VLOADA(s);
            const Vector s1 = VADD(VLOADU(s+1), VLOADU(s-1)); // Factorization due to the vertical line of symmetry
            const Vector s2 = VADD(VLOADU(s+2), VLOADU(s-2)); // Factorization due to the vertical line of symmetry
            const Vector s3 = VADD(VLOADU(s+3), VLOADU(s-3)); // Factorization due to the vertical line of symmetry
            const Vector s4 = VADD(VLOADU(s+4), VLOADU(s-4)); // Factorization due to the vertical line of symmetry
            const Vector s5 = VADD(VLOADU(s+5), VLOADU(s-5)); // Factorization due to the vertical line of symmetry

            Vector sum0 = VMUL(s0, k(0,0));
            Vector sum1 = VMUL(s0, k(0,1));
            Vector sum2 = VMUL(s0, k(0,2));
            Vector sum3 = VMUL(s0, k(0,3));
            Vector sum4 = VMUL(s0, k(0,4));
            Vector sum5 = VMUL(s0, k(0,5));

            sum0 = VFMADD(s1, k(0,1), sum0);
            sum1 = VFMADD(s1, k(1,1), sum1);
            sum2 = VFMADD(s1, k(1,2), sum2);
            sum3 = VFMADD(s1, k(1,3), sum3);
            sum4 = VFMADD(s1, k(1,4), sum4);
            sum5 = VFMADD(s1, k(1,5), sum5);

            sum0 = VFMADD(s2, k(0,2), sum0);
            sum1 = VFMADD(s2, k(1,2), sum1);
            sum2 = VFMADD(s2, k(2,2), sum2);
            sum3 = VFMADD(s2, k(2,3), sum3);
            sum4 = VFMADD(s2, k(2,4), sum4);
            sum5 = VFMADD(s2, k(2,5), sum5);

            sum0 = VFMADD(s3, k(0,3), sum0);
            sum1 = VFMADD(s3, k(1,3), sum1);
            sum2 = VFMADD(s3, k(2,3), sum2);
            sum3 = VFMADD(s3, k(3,3), sum3);
            sum4 = VFMADD(s3, k(3,4), sum4);
            sum5 = VFMADD(s3, k(3,5), sum5);

            sum0 = VFMADD(s4, k(0,4), sum0);
            sum1 = VFMADD(s4, k(1,4), sum1);
            sum2 = VFMADD(s4, k(2,4), sum2);
            sum3 = VFMADD(s4, k(3,4), sum3);
            sum4 = VFMADD(s4, k(4,4), sum4);
            sum5 = VFMADD(s4, k(4,5), sum5);

            sum0 = VFMADD(s5, k(0,5), sum0);
            sum1 = VFMADD(s5, k(1,5), sum1);
            sum2 = VFMADD(s5, k(2,5), sum2);
            sum3 = VFMADD(s5, k(3,5), sum3);
            sum4 = VFMADD(s5, k(4,5), sum4);
            sum5 = VFMADD(s5, k(5,5), sum5);

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

    _mm256_zeroupper();
}


const GaussianBlurFct g_gaussianBlurFct = gaussian_blur;


}}} // namespace rmgr::ssim::fma

#endif // __FMA__
#endif // RMGR_ARCH_IS_X86_ANY
