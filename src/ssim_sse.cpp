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

#include "ssim_simd.h"

#if !defined(__SSE__) && !RMGR_COMPILER_IS_MSVC

namespace rmgr { namespace ssim { namespace sse
{
    const GaussianBlurFct = NULL;
}    

#else

#include <cassert>
#include <cstring>
#include <xmmintrin.h>

namespace rmgr { namespace ssim { namespace sse
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
                const __m128 k128 = _mm_set1_ps(k);
                const float* s    = srce + (yd+yk) * srceStride + xk;
                __m128*      d128 = reinterpret_cast<__m128*>(dest);

                int32_t xd = width;

#if 1
                // 8x unrolled SSE loop
                while ((xd -= 8*4) >= 0)
                {
                    const __m128 sk0 = _mm_mul_ps(_mm_load_ps(s), k128);  s += 4;
                    const __m128 sk1 = _mm_mul_ps(_mm_load_ps(s), k128);  s += 4;
                    const __m128 sk2 = _mm_mul_ps(_mm_load_ps(s), k128);  s += 4;
                    const __m128 sk3 = _mm_mul_ps(_mm_load_ps(s), k128);  s += 4;
                    const __m128 sk4 = _mm_mul_ps(_mm_load_ps(s), k128);  s += 4;
                    const __m128 sk5 = _mm_mul_ps(_mm_load_ps(s), k128);  s += 4;
                    const __m128 sk6 = _mm_mul_ps(_mm_load_ps(s), k128);  s += 4;
                    const __m128 sk7 = _mm_mul_ps(_mm_load_ps(s), k128);  s += 4;
                    d128[0] = _mm_add_ps(sk0, d128[0]);
                    d128[1] = _mm_add_ps(sk1, d128[1]);
                    d128[2] = _mm_add_ps(sk2, d128[2]);
                    d128[3] = _mm_add_ps(sk3, d128[3]);
                    d128[4] = _mm_add_ps(sk4, d128[4]);
                    d128[5] = _mm_add_ps(sk5, d128[5]);
                    d128[6] = _mm_add_ps(sk6, d128[6]);
                    d128[7] = _mm_add_ps(sk7, d128[7]);
                    d128 += 8;
                }
                xd += 8*4;
#endif

#if 1
                // Epilogue SSE loop
                while ((xd -= 4) >= 0)
                {
                    *d128 = _mm_add_ps(_mm_mul_ps(_mm_load_ps(s), k128), *d128);
                    ++d128;
                    s += 4;
                }
                xd += 4;
#endif

                // Epilogue scalar loop
                float* d = reinterpret_cast<float*>(d128);
                while (--xd >= 0)
                    *d++ += *s++ * k;
            }
        }

        dest += destStride;
    }
}


const GaussianBlurFct g_gaussianBlurFct = gaussian_blur;


}}} // namespace rmgr::ssim::sse

#endif
