/*
 * Copyright (c) 2019, Romain Bailly
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

#if !defined(__AVX__)

namespace rmgr { namespace ssim { namespace avx
{
    const GaussianBlurFct = NULL;
}    

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
    #define VMUL(a,b)      _mm256_mul_pd((a), (b))
    #define VSET1(val)     _mm256_set1_pd(val)
#else
    typedef __m256         Vector;
    #define VEC_SIZE       8
    #define VCOEFF(val)    val##f, val##f, val##f, val##f, val##f, val##f, val##f, val##f
    #define VLOADA(addr)   _mm256_load_ps((addr))
    #define VLOADU(addr)   _mm256_loadu_ps((addr))
    #define VADD(a,b)      _mm256_add_ps((a), (b))
    #define VMUL(a,b)      _mm256_mul_ps((a), (b))
    #define VSET1(val)     _mm256_set1_ps(val)
#endif


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
    // Note that always process in batches of 8 because we know the buffer is large enough.

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

                // 8x unrolled SSE loop
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
    assert(radius == 5);

    // The precomputed 21 unique values of the Gaussian kernel
    static RMGR_ALIGNED_VAR(64, const Float, k21[21 * VEC_SIZE]) =
    {
        VCOEFF(0.07076223776394697),
        VCOEFF(0.05666197049168457), VCOEFF(0.04537135909566032),
        VCOEFF(0.02909122564855043), VCOEFF(0.02329443247348710), VCOEFF(0.01195976041003701),
        VCOEFF(0.00957662749024029), VCOEFF(0.00766836382523672), VCOEFF(0.00393706926284678), VCOEFF(0.00129605559384320),
        VCOEFF(0.00202135875836257), VCOEFF(0.00161857756253439), VCOEFF(0.00083100542908720), VCOEFF(0.00027356116008581), VCOEFF(0.00005774112519786),
        VCOEFF(0.00027356116008581), VCOEFF(0.00021905065286602), VCOEFF(0.00011246435511668), VCOEFF(0.00003702247708275), VCOEFF(0.00000781441153305), VCOEFF(0.00000105756559815)
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


}}} // namespace rmgr::ssim::avx

#endif
