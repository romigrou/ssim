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

#include <rmgr/ssim.h>
#include "ssim_simd.h"
#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstring>


#ifndef RMGR_SSIM_REPORT_ERROR
    #ifdef _DEBUG
        #define RMGR_SSIM_REPORT_ERROR(...)  fprintf(stderr, __VA_ARGS__)
    #else
        #define RMGR_SSIM_REPORT_ERROR(...)
    #endif
#endif


#ifndef RMGR_SSIM_TILE_ALIGNMENT
    #define RMGR_SSIM_TILE_ALIGNMENT  64
#endif


typedef float Float;



//=================================================================================================
// Allocator stuff

#if defined(_MSC_VER)
    #include <malloc.h>
    static void* default_alloc(size_t size, size_t alignment) RMGR_NOEXCEPT
    {
        return ::_aligned_malloc(size, alignment);
    }
    static void default_dealloc(void* address) RMGR_NOEXCEPT
    {
        ::_aligned_free(address);
    }
#elif _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600
    #include <stdlib.h>
    static void* default_alloc(size_t size, size_t alignment) RMGR_NOEXCEPT
    {
        void* address = NULL;
        int   err = ::posix_memalign(&address, alignment, size);
        return (err==0) ? address : NULL;
    }
    static void default_dealloc(void* address) RMGR_NOEXCEPT
    {
        ::free(address);
    }
#elif _ISOC11_SOURCE
    #include <stdlib.h>
    static void* default_alloc(size_t size, size_t alignment) RMGR_NOEXCEPT
    {
        size = (size + alignment-1) & ~(alignment - 1);
        return ::aligned_alloc(alignment, size);
    }
    static void default_dealloc(void* address) RMGR_NOEXCEPT
    {
        ::free(address);
    }
#else
    #include <stdlib.h>

    static void* default_alloc(size_t size, size_t alignment) RMGR_NOEXCEPT
    {
        assert(alignment >= sizeof(char*));

        const size_t extraSize = sizeof(char*) + alignment-1;
        char* buf = static_cast<char*>(::malloc(size + extraSize));
        if (buf == NULL)
            return NULL;

        char* alignedBuf = reinterpret_cast<char*>((reinterpret_cast<uintptr_t>(buf) + extraSize) & ~(alignment-1));
        assert(size_t(alignedBuf-buf) <= extraSize);
        assert(reinterpret_cast<uintptr_t>(alignedBuf) % sizeof(char*) == 0);
        reinterpret_cast<char**>(alignedBuf)[-1] = buf;
        return alignedBuf;
    }

    void default_dealloc(void* address) RMGR_NOEXCEPT
    {
        if (address == NULL)
            return;
        void* originalPtr = reinterpret_cast<char**>(address)[-1];
        ::free(originalPtr);
    }
#endif


namespace rmgr { namespace ssim
{


static AllocFct   g_alloc   = default_alloc;
static DeallocFct g_dealloc = default_dealloc;

static inline void* alloc(size_t size, size_t alignment=0) RMGR_NOEXCEPT
{
    assert(g_alloc != NULL);
    assert((alignment & (alignment-1)) == 0); // 0 or a power of 2
    const size_t minAlignment = sizeof(void*);
    return g_alloc(size, (alignment>=minAlignment)?alignment:minAlignment);
}

static inline void dealloc(void* address) RMGR_NOEXCEPT
{
    assert(g_dealloc != NULL);
    return g_dealloc(address);
}


static const unsigned CACHE_ALIGNMENT = 64;


static inline size_t align_up(size_t size, size_t alignment) RMGR_NOEXCEPT
{
    assert(alignment!=0 && (alignment & (alignment-1))==0); // Power of 2
    return (size + (alignment-1)) & ~(alignment-1);
}


/**
 * @brief Multiplies a tile by another tile
 */
static void multiply(Float* product, const Float* a, const Float* b, uint32_t width, uint32_t height, size_t stride) RMGR_NOEXCEPT
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


static inline Float gaussian_kernel(int x, int y, Float sigma) RMGR_NOEXCEPT
{
    Float sigma2      = sigma * sigma;
    Float numerator   = std::exp(-Float(x*x + y*y) / (2 * sigma2));
    Float denominator = Float(2*M_PI) * sigma2;
    return numerator / denominator;
}


static void precompute_gaussian_kernel(Float kernel[], int radius, Float sigma) RMGR_NOEXCEPT
{
    assert(radius != 0);

    const int size = 2*radius + 1;

    double sum = 0;

    // Compute kernel, exploiting (some) symmetries
    #define k(x,y)  kernel[(y)*size + (x)]
    for (int y=0; y<=radius; ++y)
    {
        for (int x=0; x<=radius; ++x)
            sum += k(x,y) = gaussian_kernel(x-radius, y-radius, sigma);
        for (int x=radius+1; x<size; ++x)
            sum += k(x,y) = k(size-1-x, y);
    }
    for (int y=radius+1; y<size; ++y)
        for (int x=0; x<size; ++x)
            sum += k(x,y) = k(x, size-1-y);
    #undef k

    // Normalize kernel
    for (int i = 0; i < size * size; i++)
        kernel[i] /= Float(sum);

#if 0
    printf("{\n");
    for (int y = 0; y < size; y++)
    {
        printf("  ");
        for (int x = 0; x < size; x++)
            printf("%9.7ff, ", kernel[x + y * size]);
        printf("\n");
    }
    printf("}\n\n");
#endif
}


static void gaussian_blur(Float* dest, ptrdiff_t destStride, const Float* srce, ptrdiff_t srceStride, int32_t width, int32_t height, const Float kernel[], int radius) RMGR_NOEXCEPT
{
    assert(width  > 0);
    assert(height > 0);
    assert(srceStride >= destStride + 2*radius);

    const int32_t kernelStride = 2*radius + 1;
    for (int32_t yd=0; yd<height; ++yd)
    {
        memset(dest, 0, width*sizeof(Float));
        for (int32_t yk=-radius; yk<=radius; ++yk)
        {
            const Float* kernelRow = kernel + yk * kernelStride;
            for (int32_t xk=-radius; xk<=radius; ++xk)
            {
                const Float  k = kernelRow[xk];
                const Float* s = srce + (yd+yk) * srceStride + xk;
                Float*       d = dest;

                for (int32_t xd=0; xd<width; ++xd)
                    *d++ += *s++ * k;
            }
        }

        dest += destStride;
    }
}


static void retrieve_tile(Float* tile, uint32_t tileWidth, uint32_t tileHeight, size_t tileStride, uint32_t margin, uint32_t x, uint32_t y,
                          const uint8_t* imgData, uint32_t imgWidth, uint32_t imgHeight, ptrdiff_t imgStep, ptrdiff_t imgStride) RMGR_NOEXCEPT
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

    const uint8_t* s       = imgData + sx1 * imgStep + sy1 * imgStride;
    Float*         d       = tile + tileStride * (sy1 - dy1);
    const size_t   sStride = imgStride  - imgStep * (sx2 - sx1);
    const size_t   dStride = tileStride - (tileWidth + 2*margin);
    for (int32_t dy=sy1; dy<sy2; ++dy)
    {
        // Left margin
        for (int32_t dx=dx1; dx<sx1; ++dx)
            *d++ = Float(*s);

        // Actual data
        for (int32_t dx=sx1; dx<sx2; ++dx)
        {
            *d++ = Float(*s);
            s += imgStep;
        }

        // Right margin
        const Float right = d[-1];
        for (int32_t dx=sx2; dx<dx2; ++dx)
            *d++ = right;

        s += sStride;
        d += dStride;
    }

    // Bottom margin
    const size_t tileRowSize = (tileWidth + 2*margin) * sizeof(Float);
    if (dy2 > sy2)
    {
        Float* lastRow = d - tileStride;
        for (int32_t dy=sy2; dy<dy2; ++dy)
        {
            memcpy(d, lastRow, tileRowSize);
            d += tileStride;
        }
    }

    // Top margin
    if (dy1 < sy1)
    {
        Float* firstRow = tile + tileStride * (sy1 - dy1);
        d = tile;
        for (int32_t dy=dy1; dy<sy1; ++dy)
        {
            memcpy(d , firstRow, tileRowSize);
            d += tileStride;
        }
    }
}


volatile GaussianBlurFct g_gaussianBlurFct = NULL;


//=================================================================================================
// x86 init

#if RMGR_ARCH_IS_X86_ANY



#ifdef _MSC_VER
    #include <intrin.h>
    static inline void cpu_id(int leaf, int regs[4]) RMGR_NOEXCEPT
    {
        __cpuid(regs, leaf);
    }
#else
    #include <cpuid.h>
    static inline void cpu_id(int leaf, int regs[4]) RMGR_NOEXCEPT
    {
        __get_cpuid(leaf, reinterpret_cast<unsigned*>(regs), reinterpret_cast<unsigned*>(regs+1), reinterpret_cast<unsigned*>(regs+2), reinterpret_cast<unsigned*>(regs+3));
    }
#endif


static void init_x86() RMGR_NOEXCEPT
{
    // Detect machine's features
    int regs[4] = {};
    cpu_id(0, regs);
    const uint32_t maxLeaf = regs[0];
    if (maxLeaf >= 1)
    {
        cpu_id(1, regs);
        const uint32_t ecx = regs[2];
        const uint32_t edx = regs[3];

        if ((ecx & (1 << 28))!=0 && avx::g_gaussianBlurFct!=NULL)
            g_gaussianBlurFct = avx::g_gaussianBlurFct;
        else if ((edx & (1 << 25))!=0 && sse::g_gaussianBlurFct!=NULL)
            g_gaussianBlurFct = sse::g_gaussianBlurFct;
        else
            g_gaussianBlurFct = gaussian_blur;
    }
}

    

#endif // RMGR_ARCH_IS_X86_ANY


//=================================================================================================
// Main function

float compute_ssim(uint32_t width, uint32_t height,
                   const uint8_t* imgAData, ptrdiff_t imgAStep, ptrdiff_t imgAStride,
                   const uint8_t* imgBData, ptrdiff_t imgBStep, ptrdiff_t imgBStride,
                   float* ssimMap, ptrdiff_t ssimStep, ptrdiff_t ssimStride) RMGR_NOEXCEPT
{
    GaussianBlurFct gaussianBlur = g_gaussianBlurFct;
    if (gaussianBlur == NULL)
    {
#if RMGR_ARCH_IS_X86_ANY
        init_x86();
#else
        g_gaussianBlurFct = gaussian_blur;
#endif
        gaussianBlur = g_gaussianBlurFct;
    }

    const Float k1 = Float(0.01);
    const Float k2 = Float(0.03);
    const Float L  = UINT8_MAX;
    const Float c1 = (k1 * L) * (k1 * L);
    const Float c2 = (k2 * L) * (k2 * L);

    if (imgAData==NULL || imgBData==NULL)
    {
        RMGR_SSIM_REPORT_ERROR("Invalid parameter: imgAData or imgBData is NULL\n");
        return -EINVAL;
    }

    if (ssimMap == NULL)
    {
        ssimStep   = 0;
        ssimStride = 0;
    }

    // Parameters of the gaussian blur
    const unsigned radius = 5;
    const Float    sigma  = 1.5;
    Float kernelBuffer[(2*radius+1) * (2*radius+1)];
    precompute_gaussian_kernel(kernelBuffer, radius, sigma);

    const Float* kernel = kernelBuffer + radius * (2*radius+1) + radius; // Center of the kernel

    const uint32_t tileSize   = 64;
    const uint32_t tileStride = tileSize + 2 *radius;

    double sum = 0.0;
    for (uint32_t ty=0; ty<height; ty+=tileSize)
    {
        const uint32_t th = std::min(tileSize, height-ty);
        for (uint32_t tx=0; tx<width; tx+=tileSize)
        {
            const uint32_t tw = std::min(tileSize, width-tx);

            RMGR_ALIGNED(RMGR_SSIM_TILE_ALIGNMENT) Float buffer1[tileSize   * tileSize];   // Yes, this one needs not be as large as the other ones
            RMGR_ALIGNED(RMGR_SSIM_TILE_ALIGNMENT) Float buffer2[tileStride * tileStride];
            RMGR_ALIGNED(RMGR_SSIM_TILE_ALIGNMENT) Float buffer3[tileStride * tileStride];
            RMGR_ALIGNED(RMGR_SSIM_TILE_ALIGNMENT) Float buffer4[tileStride * tileStride];
            RMGR_ALIGNED(RMGR_SSIM_TILE_ALIGNMENT) Float buffer5[tileStride * tileStride];
            RMGR_ALIGNED(RMGR_SSIM_TILE_ALIGNMENT) Float buffer6[tileStride * tileStride];

            Float* a = buffer2;
            Float* b = buffer3;
            retrieve_tile(a, tw, th, tileStride, radius, tx, ty, imgAData, width, height, imgAStep, imgAStride);
            retrieve_tile(b, tw, th, tileStride, radius, tx, ty, imgBData, width, height, imgBStep, imgBStride);

            Float* a2 = buffer4;
            Float* b2 = buffer5;
            Float* ab = buffer6;
            multiply(a2, a, a, tw+2*radius, th+2*radius, tileStride);
            multiply(b2, b, b, tw+2*radius, th+2*radius, tileStride);
            multiply(ab, a, b, tw+2*radius, th+2*radius, tileStride);

            Float* muATile     = buffer1;
            Float* muBTile     = buffer2;
            Float* sigmaA2Tile = buffer3;
            Float* sigmaB2Tile = buffer4;
            Float* sigmaABTile = buffer5;
            gaussianBlur(muATile,     tileSize, a  + radius*tileStride + radius, tileStride, tw, th, kernel, radius);
            gaussianBlur(muBTile,     tileSize, b  + radius*tileStride + radius, tileStride, tw, th, kernel, radius);
            gaussianBlur(sigmaA2Tile, tileSize, a2 + radius*tileStride + radius, tileStride, tw, th, kernel, radius);
            gaussianBlur(sigmaB2Tile, tileSize, b2 + radius*tileStride + radius, tileStride, tw, th, kernel, radius);
            gaussianBlur(sigmaABTile, tileSize, ab + radius*tileStride + radius, tileStride, tw, th, kernel, radius);

            float* ssimTile = ssimMap + tx * ssimStep + ty * ssimStride;

            double tileSum = 0.0f;
            for (uint32_t y=0; y<th; ++y)
            {
                const Float* muARow     = muATile     + y * tileSize;
                const Float* muBRow     = muBTile     + y * tileSize;
                const Float* sigmaA2Row = sigmaA2Tile + y * tileSize;
                const Float* sigmaB2Row = sigmaB2Tile + y * tileSize;
                const Float* sigmaABRow = sigmaABTile + y * tileSize;
                float*       ssimPtr    = ssimTile    + y * ssimStride;
                for (uint32_t x=0; x<tw; ++x)
                {
                    const double muA     = muARow[x];
                    const double muB     = muBRow[x];
                    const double muA2    = muA * muA;
                    const double muB2    = muB * muB;
                    const double muAB    = muA * muB;
                    const double sigmaA2 = sigmaA2Row[x] - muA2;
                    const double sigmaB2 = sigmaB2Row[x] - muB2;
                    const double sigmaAB = sigmaABRow[x] - muAB;

                    const double numerator1   = 2 * muAB    + c1;
                    const double numerator2   = 2 * sigmaAB + c2;
                    const double denominator1 = muA2 + muB2 + c1;
                    const double denominator2 = sigmaA2 + sigmaB2 + c2;

                    const double numerator   = numerator1 * numerator2;
                    const double denominator = denominator1 * denominator2;

                    const double ssim = numerator / denominator;
                    tileSum += ssim;

                    if (ssimMap != NULL)
                    {
                        *ssimPtr = float(ssim);
                        ssimPtr += ssimStep;
                    }
                }
            }

            sum += tileSum;
        }
    }

    return float(sum / double(width * height));
}


}} // namespace rmgr::ssim
