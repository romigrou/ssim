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

#if !defined(__AVX__) && RMGR_COMPILER_IS_MSVC_AT_LEAST(16,0,0)
    #define __AVX__  1
#endif


#if !defined(__AVX__)

namespace rmgr { namespace ssim { namespace avx
{
    const MultiplyFct     g_multiplyFct     = NULL;
    const GaussianBlurFct g_gaussianBlurFct = NULL;
    const SumTileFct      g_sumTileFct      = NULL;
}}}

#else

#include <cassert>
#include <cstring>
#include <immintrin.h>

namespace rmgr { namespace ssim { namespace avx
{

#if RMGR_SSIM_USE_DOUBLE
    typedef __m256d        Vector;
    #define VEC_SIZE       4
    #define VCOEFF(val)    val, val, val, val
    #define VLOADA(addr)   _mm256_load_pd((addr))
    #define VLOADU(addr)   _mm256_loadu_pd((addr))
    #define VADD(a,b)      _mm256_add_pd((a), (b))
    #define VSUB(a,b)      _mm256_sub_pd((a), (b))
    #define VMUL(a,b)      _mm256_mul_pd((a), (b))
    #define VDIV(a,b)      _mm256_div_pd((a), (b))
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
    #define VDIV(a,b)      _mm256_div_ps((a), (b))
    #define VSET1(val)     _mm256_set1_ps(val)
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

        // AVX loop. Note that we always process in batches of 4 because we know the buffer is large enough.
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
        }
        while ((x -= 4*VEC_SIZE) >= 0);

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
    assert(reinterpret_cast<uintptr_t>(srce) % (VEC_SIZE * sizeof(Float)) == 0);
    assert(reinterpret_cast<uintptr_t>(dest) % (VEC_SIZE * sizeof(Float)) == 0);

    const size_t rowSize = width * sizeof(Float);

#if 0
    // Generic AVX implementation.
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

                // 8x unrolled AVX loop
                while ((xd -= 8*VEC_SIZE) >= 0)
                {
                    const Vector sk0 = VMUL(VLOADU(s), k);  s += VEC_SIZE;
                    const Vector sk1 = VMUL(VLOADU(s), k);  s += VEC_SIZE;
                    const Vector sk2 = VMUL(VLOADU(s), k);  s += VEC_SIZE;
                    const Vector sk3 = VMUL(VLOADU(s), k);  s += VEC_SIZE;
                    const Vector sk4 = VMUL(VLOADU(s), k);  s += VEC_SIZE;
                    const Vector sk5 = VMUL(VLOADU(s), k);  s += VEC_SIZE;
                    const Vector sk6 = VMUL(VLOADU(s), k);  s += VEC_SIZE;
                    const Vector sk7 = VMUL(VLOADU(s), k);  s += VEC_SIZE;
                    vd[0] = VADD(sk0, vd[0]);
                    vd[1] = VADD(sk1, vd[1]);
                    vd[2] = VADD(sk2, vd[2]);
                    vd[3] = VADD(sk3, vd[3]);
                    vd[4] = VADD(sk4, vd[4]);
                    vd[5] = VADD(sk5, vd[5]);
                    vd[6] = VADD(sk6, vd[6]);
                    vd[7] = VADD(sk7, vd[7]);
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

            sum0 = VADD(sum0, VMUL(s1, k(0,1)));
            sum1 = VADD(sum1, VMUL(s1, k(1,1)));
            sum2 = VADD(sum2, VMUL(s1, k(1,2)));
            sum3 = VADD(sum3, VMUL(s1, k(1,3)));
            sum4 = VADD(sum4, VMUL(s1, k(1,4)));
            sum5 = VADD(sum5, VMUL(s1, k(1,5)));

            sum0 = VADD(sum0, VMUL(s2, k(0,2)));
            sum1 = VADD(sum1, VMUL(s2, k(1,2)));
            sum2 = VADD(sum2, VMUL(s2, k(2,2)));
            sum3 = VADD(sum3, VMUL(s2, k(2,3)));
            sum4 = VADD(sum4, VMUL(s2, k(2,4)));
            sum5 = VADD(sum5, VMUL(s2, k(2,5)));

            sum0 = VADD(sum0, VMUL(s3, k(0,3)));
            sum1 = VADD(sum1, VMUL(s3, k(1,3)));
            sum2 = VADD(sum2, VMUL(s3, k(2,3)));
            sum3 = VADD(sum3, VMUL(s3, k(3,3)));
            sum4 = VADD(sum4, VMUL(s3, k(3,4)));
            sum5 = VADD(sum5, VMUL(s3, k(3,5)));

            sum0 = VADD(sum0, VMUL(s4, k(0,4)));
            sum1 = VADD(sum1, VMUL(s4, k(1,4)));
            sum2 = VADD(sum2, VMUL(s4, k(2,4)));
            sum3 = VADD(sum3, VMUL(s4, k(3,4)));
            sum4 = VADD(sum4, VMUL(s4, k(4,4)));
            sum5 = VADD(sum5, VMUL(s4, k(4,5)));

            sum0 = VADD(sum0, VMUL(s5, k(0,5)));
            sum1 = VADD(sum1, VMUL(s5, k(1,5)));
            sum2 = VADD(sum2, VMUL(s5, k(2,5)));
            sum3 = VADD(sum3, VMUL(s5, k(3,5)));
            sum4 = VADD(sum4, VMUL(s5, k(4,5)));
            sum5 = VADD(sum5, VMUL(s5, k(5,5)));

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

double sum_tile(uint32_t tileWidth, uint32_t tileHeight, uint32_t tileStride, Float c1, Float c2,
                const Float* muATile, const Float* muBTile, const Float* sigmaA2Tile, const Float* sigmaB2Tile, const Float* sigmaABTile,
                float* ssimTile, ptrdiff_t ssimStep, ptrdiff_t ssimStride) RMGR_NOEXCEPT
{
#if RMGR_SSIM_USE_DOUBLE
    assert(ssimTile == NULL);
    (void)ssimTile;
    (void)ssimStep;
    (void)ssimStride;
#else
    assert(ssimTile==NULL || ssimStep==1);
    (void)ssimStep;
#endif

    const Vector vc1  = VSET1( c1);
    const Vector vnc2 = VSET1(-c2); // Because we compute -sigma instead of sigma (but the numerator and denominator signs cancel out)

    double tileSum = 0.0f; // The sum is always done on a double to increase precision

    for (uint32_t y=0; y<tileHeight; ++y)
    {
        const Float* muARow     = muATile     + y * tileStride;
        const Float* muBRow     = muBTile     + y * tileStride;
        const Float* sigmaA2Row = sigmaA2Tile + y * tileStride;
        const Float* sigmaB2Row = sigmaB2Tile + y * tileStride;
        const Float* sigmaABRow = sigmaABTile + y * tileStride;
#if !RMGR_SSIM_USE_DOUBLE
        float*       ssimPtr    = ssimTile    + y * ssimStride;
#endif

        int32_t x = tileWidth;

        // AVX loop
        if ((x -= VEC_SIZE) >= 0)
        {
#if RMGR_SSIM_USE_DOUBLE
            __m256d rowSum   = _mm256_set1_pd(0.0);
#else
            __m256d rowSumLo = _mm256_set1_pd(0.0);
            __m256d rowSumHi = _mm256_set1_pd(0.0);
#endif
            do
            {
                const Vector muA         = VLOADA(muARow);  muARow+=VEC_SIZE;
                const Vector muB         = VLOADA(muBRow);  muBRow+=VEC_SIZE;
                const Vector muA2        = VMUL(muA, muA);
                const Vector muB2        = VMUL(muB, muB);
                const Vector muAB        = VMUL(muA, muB);
                const Vector sigmaA2     = VSUB(muA2, VLOADA(sigmaA2Row));  sigmaA2Row+=VEC_SIZE;
                const Vector sigmaB2     = VSUB(muB2, VLOADA(sigmaB2Row));  sigmaB2Row+=VEC_SIZE;
                const Vector sigmaAB     = VSUB(muAB, VLOADA(sigmaABRow));  sigmaABRow+=VEC_SIZE;
                const Vector numerator   = VMUL(VADD(VADD(muAB,muAB), vc1), VADD(VADD(sigmaAB,sigmaAB), vnc2));
                const Vector denominator = VMUL(VADD(VADD(muA2,muB2), vc1), VADD(VADD(sigmaA2,sigmaB2), vnc2));
                const Vector ssim        = VDIV(numerator, denominator);

#if RMGR_SSIM_USE_DOUBLE
                rowSum   = _mm256_add_pd(rowSum, ssim);
#else
                rowSumLo = _mm256_add_pd(rowSumLo, _mm256_cvtps_pd(_mm256_castps256_ps128(ssim)));
                rowSumHi = _mm256_add_pd(rowSumHi, _mm256_cvtps_pd(_mm256_extractf128_ps(ssim, 1)));
#endif

#if !RMGR_SSIM_USE_DOUBLE
                if (ssimTile != NULL)
                {
                    _mm256_storeu_ps(ssimPtr, ssim);
                    ssimPtr += VEC_SIZE;
                }
#endif
            }
            while ((x -= VEC_SIZE) >= 0);

#if !RMGR_SSIM_USE_DOUBLE
            const __m256d rowSum  = _mm256_add_pd(rowSumLo, rowSumHi);
#endif
            const __m128d rowSum2 = _mm_add_pd(_mm256_castpd256_pd128(rowSum), _mm256_extractf128_pd(rowSum,1));
            tileSum += _mm_cvtsd_f64(rowSum2);
            tileSum += _mm_cvtsd_f64(_mm_shuffle_pd(rowSum2, rowSum2, _MM_SHUFFLE2(0,1)));
        }
        x += VEC_SIZE;

        // Scalar epilogue
        while (--x >= 0)
        {
            const Float muA     = *muARow++;
            const Float muB     = *muBRow++;
            const Float muA2    = muA * muA;
            const Float muB2    = muB * muB;
            const Float muAB    = muA * muB;
            const Float sigmaA2 = *sigmaA2Row++ - muA2;
            const Float sigmaB2 = *sigmaB2Row++ - muB2;
            const Float sigmaAB = *sigmaABRow++ - muAB;

            const Float numerator   = (2 * muAB    + c1) * (2 * sigmaAB + c2);
            const Float denominator = (muA2 + muB2 + c1) * (sigmaA2 + sigmaB2 + c2);
            const Float ssim        = numerator / denominator;
            tileSum += ssim;

#if !RMGR_SSIM_USE_DOUBLE
            if (ssimTile != NULL)
            {
                *ssimPtr = float(ssim);
                ssimPtr += ssimStep;
            }
#endif
        }

    }

    return tileSum;
}

const SumTileFct g_sumTileFct = sum_tile;


}}} // namespace rmgr::ssim::avx

#endif
