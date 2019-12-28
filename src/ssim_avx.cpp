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

static void gaussian_blur(float* dest, ptrdiff_t destStride, const float* srce, ptrdiff_t srceStride, int32_t width, int32_t height, const float kernel[], int radius) RMGR_NOEXCEPT
{
    assert(width  > 0);
    assert(height > 0);
    assert(srceStride >= destStride + 2*radius);

    const int32_t kernelStride = 2*radius + 1;
    const size_t  rowSize      = width * sizeof(float);

    for (int32_t yd=0; yd<height; ++yd)
    {
        memset(dest, 0, rowSize);
        for (int32_t yk=-radius; yk<=radius; ++yk)
        {
            const float* kernelRow = kernel + yk * kernelStride;
            for (int32_t xk=-radius; xk<=radius; ++xk)
            {
                const float  k    = kernelRow[xk];
                const __m256 k256 = _mm256_set1_ps(k);
                const float* s    = srce + (yd+yk) * srceStride + xk;
                __m256*      d256 = reinterpret_cast<__m256*>(dest);

                int32_t xd = width;

#if 1
                // 8x unrolled AVX loop
                while ((xd -= 8*8) >= 0)
                {
                    const __m256 sk0 = _mm256_mul_ps(_mm256_loadu_ps(s), k256);  s += 8;
                    const __m256 sk1 = _mm256_mul_ps(_mm256_loadu_ps(s), k256);  s += 8;
                    const __m256 sk2 = _mm256_mul_ps(_mm256_loadu_ps(s), k256);  s += 8;
                    const __m256 sk3 = _mm256_mul_ps(_mm256_loadu_ps(s), k256);  s += 8;
                    const __m256 sk4 = _mm256_mul_ps(_mm256_loadu_ps(s), k256);  s += 8;
                    const __m256 sk5 = _mm256_mul_ps(_mm256_loadu_ps(s), k256);  s += 8;
                    const __m256 sk6 = _mm256_mul_ps(_mm256_loadu_ps(s), k256);  s += 8;
                    const __m256 sk7 = _mm256_mul_ps(_mm256_loadu_ps(s), k256);  s += 8;
                    d256[0] = _mm256_add_ps(sk0, d256[0]);
                    d256[1] = _mm256_add_ps(sk1, d256[1]);
                    d256[2] = _mm256_add_ps(sk2, d256[2]);
                    d256[3] = _mm256_add_ps(sk3, d256[3]);
                    d256[4] = _mm256_add_ps(sk4, d256[4]);
                    d256[5] = _mm256_add_ps(sk5, d256[5]);
                    d256[6] = _mm256_add_ps(sk6, d256[6]);
                    d256[7] = _mm256_add_ps(sk7, d256[7]);
                    d256 += 8;
                }
                xd += 8*8;
#endif

#if 1
                // Epilogue AVX loop
                while ((xd -= 8) >= 0)
                {
                    *d256 = _mm256_add_ps(_mm256_mul_ps(_mm256_loadu_ps(s), k256), *d256);
                    ++d256;
                    s += 8;
                }
                xd += 8;
#endif

                // Epilogue scalar loop
                float* d = reinterpret_cast<float*>(d256);
                while (--xd >= 0)
                    *d++ += *s++ * k;
            }
        }

        dest += destStride;
    }
}


const GaussianBlurFct g_gaussianBlurFct = gaussian_blur;


}}} // namespace rmgr::ssim::avx

#endif
