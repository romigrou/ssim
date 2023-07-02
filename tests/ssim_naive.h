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

#include <rmgr/ssim.h>
#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#if defined(__SIZEOF_FLOAT128__) && RMGR_COMPILER_IS_GCC
    #include <quadmath.h>
#endif


namespace rmgr { namespace ssim { namespace naive
{


/**
 * @brief Multiplies a tile by another tile
 */
template<typename F>
void multiply(F* product, const F* a, const F* b, uint32_t width, uint32_t height, size_t stride) RMGR_NOEXCEPT
{
    for (uint32_t y=0; y<height; ++y)
    {
        for (uint32_t x=0; x<width; ++x)
            *product++ = *a++ * *b++;
        a       += stride - width;
        b       += stride - width;
        product += stride - width;
    }
}


template<typename T>
inline T exp(T x) RMGR_NOEXCEPT
{
    return std::exp(x);
}

#if defined(__SIZEOF_FLOAT128__) && RMGR_COMPILER_IS_GCC
inline __float128 exp(__float128 x)
{
    return ::expq(x);
}
#endif


template<typename F>
inline F gaussian_kernel(int x, int y, F sigma) RMGR_NOEXCEPT
{
#ifdef M_PIq
    const F tau = F(2 * M_PIq);
#elif defined(M_PIl)
    const F tau = F(2 * M_PIl);
#elif defined(M_PI)
    const F tau = F(2 * M_PI);
#else
    const F tau = F(6.283185307179586476925286766559L);
#endif

    F sigma2      = sigma * sigma;
    F numerator   = exp(-F(x*x + y*y) / (2 * sigma2));
    F denominator = tau * sigma2;
    return numerator / denominator;
}


template<typename F>
void precompute_gaussian_kernel(F kernel[], int radius, F sigma) RMGR_NOEXCEPT
{
    assert(radius != 0);

    const int size = 2*radius + 1;

    double sum = 0;

    // Compute kernel, exploiting (some) symmetries
    #define k(x,y)  kernel[(y)*size + (x)]
    for (int y=0; y<=radius; ++y)
    {
        for (int x=0; x<=radius; ++x)
            sum += double(k(x,y) = gaussian_kernel(x-radius, y-radius, sigma));
        for (int x=radius+1; x<size; ++x)
            sum += double(k(x,y) = k(size-1-x, y));
    }
    for (int y=radius+1; y<size; ++y)
        for (int x=0; x<size; ++x)
            sum += double(k(x,y) = k(x, size-1-y));
    #undef k

    // Normalize kernel
    for (int i = 0; i < size * size; i++)
        kernel[i] /= F(sum);

#if 0
    printf("{\n");
    for (int y = 0; y < size; y++)
    {
        printf("  ");
        for (int x = 0; x < size; x++)
            printf("%19.17e, ", double(kernel[x + y * size]));
        printf("\n");
    }
    printf("}\n\n");
#endif
}


template<typename F>
void gaussian_blur(F* dest, ptrdiff_t destStride, const F* srce, ptrdiff_t srceStride, int32_t width, int32_t height, const F kernel[], int radius) RMGR_NOEXCEPT
{
    assert(width  > 0);
    assert(height > 0);
    assert(srceStride >= destStride + 2*radius);

    for (int32_t yd=0; yd<height; ++yd)
    {
        for (int32_t xd=0; xd<width; ++xd)
        {
            F val = 0;
            const F* k = kernel;
            for (int32_t ys=yd-radius; ys<=yd+radius; ++ys)
            {
                const F* row = srce + ys * srceStride;
                for (int32_t xs=xd-radius; xs<=xd+radius; ++xs)
                    val += *k++ * row[xs];
            }
            *dest++ = val;
        }
        dest += destStride - width;
    }
}


template<typename F, typename T>
void retrieve_tile(F* tile, uint32_t tileWidth, uint32_t tileHeight, size_t tileStride, uint32_t margin, uint32_t x, uint32_t y,
                   const T* imgData, uint32_t imgWidth, uint32_t imgHeight, ptrdiff_t imgStep, ptrdiff_t imgStride) RMGR_NOEXCEPT
{
    assert(tileStride >= tileWidth + 2*margin);
    assert(0<=x && x<imgWidth);
    assert(0<=y && y<imgHeight);

    const int32_t dx1 = x - margin;
    const int32_t dy1 = y - margin;
    const int32_t dx2 = x + tileWidth  + margin;
    const int32_t dy2 = y + tileHeight + margin;
    const int32_t sx1 = std::max(dx1, 0);
    const int32_t sy1 = std::max(dy1, 0);
    const int32_t sx2 = std::min(dx2, int32_t(imgWidth));
    const int32_t sy2 = std::min(dy2, int32_t(imgHeight));

    const T*     s       = imgData + sx1 * imgStep + sy1 * imgStride;
    F*           d       = tile + tileStride * (sy1 - dy1);
    const size_t sStride = imgStride  - imgStep * (sx2 - sx1);
    const size_t dStride = tileStride - (tileWidth + 2*margin);
    for (int32_t dy=sy1; dy<sy2; ++dy)
    {
        // Left margin
        for (int32_t dx=dx1; dx<sx1; ++dx)
            *d++ = F(*s);

        // Actual data
        for (int32_t dx=sx1; dx<sx2; ++dx)
        {
            *d++ = F(*s);
            s += imgStep;
        }

        // Right margin
        const F right = d[-1];
        for (int32_t dx=sx2; dx<dx2; ++dx)
            *d++ = right;

        s += sStride;
        d += dStride;
    }

    // Bottom margin
    const size_t tileRowSize = (tileWidth + 2*margin) * sizeof(F);
    if (dy2 > sy2)
    {
        F* lastRow = d - tileStride;
        for (int32_t dy=sy2; dy<dy2; ++dy)
        {
            memcpy(d, lastRow, tileRowSize);
            d += tileStride;
        }
    }

    // Top margin
    if (dy1 < sy1)
    {
        F* firstRow = tile + tileStride * (sy1 - dy1);
        d = tile;
        for (int32_t dy=dy1; dy<sy1; ++dy)
        {
            memcpy(d , firstRow, tileRowSize);
            d += tileStride;
        }
    }
}


/**
 * @brief Naive SSIM implementation used for reference in unit tests
 *
 * @tparam F The floating-point type to use. This will be used for all computations.
 * @tparam T The image's data type
 */
template<typename F, typename T>
F compute_ssim(uint32_t width, uint32_t height,
               const T* imgAData, ptrdiff_t imgAStep, ptrdiff_t imgAStride,
               const T* imgBData, ptrdiff_t imgBStep, ptrdiff_t imgBStride,
               F* ssimMap, ptrdiff_t ssimStep, ptrdiff_t ssimStride) RMGR_NOEXCEPT
{
    const double k1 = 0.01;
    const double k2 = 0.03;
    const double L  = double(std::numeric_limits<T>::max());
    const F      c1 = F((k1 * L) * (k1 * L));
    const F      c2 = F((k2 * L) * (k2 * L));

    if (imgAData==NULL || imgBData==NULL)
        return F(-EINVAL);

    if (ssimMap == NULL)
    {
        ssimStep   = 0;
        ssimStride = 0;
    }

    // Parameters of the gaussian blur
    const unsigned radius = 5;
    const F        sigma  = F(1.5);
    F kernel[(2*radius+1) * (2*radius+1)];
    precompute_gaussian_kernel(kernel, radius, sigma);

    const uint32_t tileSize   = 64;
    const uint32_t tileStride = tileSize + 2 *radius;

    F sum = 0;
    for (uint32_t ty=0; ty<height; ty+=tileSize)
    {
        const uint32_t th = std::min(tileSize, height-ty);
        for (uint32_t tx=0; tx<width; tx+=tileSize)
        {
            const uint32_t tw = std::min(tileSize, width-tx);

            F buffer1[tileSize   * tileSize];   // Yes, this one needs not be as large as the other ones
            F buffer2[tileStride * tileStride];
            F buffer3[tileStride * tileStride];
            F buffer4[tileStride * tileStride];
            F buffer5[tileStride * tileStride];
            F buffer6[tileStride * tileStride];

            F* a = buffer2;
            F* b = buffer3;
            retrieve_tile(a, tw, th, tileStride, radius, tx, ty, imgAData, width, height, imgAStep, imgAStride);
            retrieve_tile(b, tw, th, tileStride, radius, tx, ty, imgBData, width, height, imgBStep, imgBStride);

            F* a2 = buffer4;
            F* b2 = buffer5;
            F* ab = buffer6;
            multiply(a2, a, a, tw+2*radius, th+2*radius, tileStride);
            multiply(b2, b, b, tw+2*radius, th+2*radius, tileStride);
            multiply(ab, a, b, tw+2*radius, th+2*radius, tileStride);

            F* muATile     = buffer1;
            F* muBTile     = buffer2;
            F* sigmaA2Tile = buffer3;
            F* sigmaB2Tile = buffer4;
            F* sigmaABTile = buffer5;
            gaussian_blur(muATile,     tileSize, a  + radius*tileStride + radius, tileStride, tw, th, kernel, radius);
            gaussian_blur(muBTile,     tileSize, b  + radius*tileStride + radius, tileStride, tw, th, kernel, radius);
            gaussian_blur(sigmaA2Tile, tileSize, a2 + radius*tileStride + radius, tileStride, tw, th, kernel, radius);
            gaussian_blur(sigmaB2Tile, tileSize, b2 + radius*tileStride + radius, tileStride, tw, th, kernel, radius);
            gaussian_blur(sigmaABTile, tileSize, ab + radius*tileStride + radius, tileStride, tw, th, kernel, radius);

            F* ssimTile = ssimMap + tx * ssimStep + ty * ssimStride;

            F tileSum = 0;
            for (uint32_t y=0; y<th; ++y)
            {
                const F* muARow     = muATile     + y * tileSize;
                const F* muBRow     = muBTile     + y * tileSize;
                const F* sigmaA2Row = sigmaA2Tile + y * tileSize;
                const F* sigmaB2Row = sigmaB2Tile + y * tileSize;
                const F* sigmaABRow = sigmaABTile + y * tileSize;
                F*       ssimPtr    = ssimTile    + y * ssimStride;
                for (uint32_t x=0; x<tw; ++x)
                {
                    const F muA     = muARow[x];
                    const F muB     = muBRow[x];
                    const F muA2    = muA * muA;
                    const F muB2    = muB * muB;
                    const F muAB    = muA * muB;
                    const F sigmaA2 = sigmaA2Row[x] - muA2;
                    const F sigmaB2 = sigmaB2Row[x] - muB2;
                    const F sigmaAB = sigmaABRow[x] - muAB;

                    const F numerator   = (2 * muAB + c1) * (2 * sigmaAB + c2);
                    const F denominator = (muA2 + muB2 + c1) * (sigmaA2 + sigmaB2 + c2);

                    const F ssim = numerator / denominator;
                    tileSum += ssim;

                    if (ssimMap != NULL)
                    {
                        *ssimPtr = ssim;
                        ssimPtr += ssimStep;
                    }
                }
            }

            sum += tileSum;
        }
    }

    return sum / F(width * height);
}


}}} // namespace rmgr::ssim::naive
