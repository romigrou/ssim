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
#include "ssim_internal.h"
#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdio>
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

#define ALIGN_UP(size, alignment)  (((size) + (alignment)-1) & ~size_t((alignment)-1))


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
    assert(srceStride >= width + 2*radius);

#if 0

    // Naive but generic implementation, kept for reference
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

#else

    // Faster, but tailored for the radius==5 case
    assert(radius == 5);
    const int32_t kernelStride = 11;

    const Float* s  = srce -  5*srceStride;
    Float*       d  = dest - 10*destStride;

    #define k(x,y)  kernel[(y) * kernelStride + (x)]

    int32_t yd = height + 2*radius;
    do
    {
        memset(d+10*destStride, 0, width*sizeof(Float));
        int32_t xd = width;
        do
        {
            Float* pd = d;
            Float dA = *pd;  pd+=destStride;
            Float dB = *pd;  pd+=destStride;
            Float dC = *pd;  pd+=destStride;
            Float dD = *pd;  pd+=destStride;
            Float dE = *pd;  pd+=destStride;
            Float dF = *pd;  pd+=destStride;
            Float dG = *pd;  pd+=destStride;
            Float dH = *pd;  pd+=destStride;
            Float dI = *pd;  pd+=destStride;
            Float dJ = *pd;  pd+=destStride;
            Float dK = *pd;  pd = d;

#if 0
            // Non-unrolled code that exploits no symmetry
            for (int32_t xk=-radius; xk<=radius; ++xk)
            {
                const float ss  = s[xk];
                const float ks0 = ss * k0[xk];
                const float ks1 = ss * k1[xk];
                const float ks2 = ss * k2[xk];
                const float ks3 = ss * k3[xk];
                const float ks4 = ss * k4[xk];
                const float ks5 = ss * k5[xk];
                dA += ks5;
                dB += ks4;
                dC += ks3;
                dD += ks2;
                dE += ks1;
                dF += ks0;
                dG += ks1;
                dH += ks2;
                dI += ks3;
                dJ += ks4;
                dK += ks5;
            }
#else
            // Unrolled code that exploits the horizontal and vertical symmetries in the kernel
            const float s0  = s[0];
            const float s1  = s[1] + s[-1];
            const float s2  = s[2] + s[-2];
            const float s3  = s[3] + s[-3];
            const float s4  = s[4] + s[-4];
            const float s5  = s[5] + s[-5];

            const float s0k0 = s0 * k(0,0);
            const float s0k1 = s0 * k(1,0);
            const float s0k2 = s0 * k(2,0);
            const float s0k3 = s0 * k(3,0);
            const float s0k4 = s0 * k(4,0);
            const float s0k5 = s0 * k(5,0);
            dA += s0k5;
            dB += s0k4;
            dC += s0k3;
            dD += s0k2;
            dE += s0k1;
            dF += s0k0;
            dG += s0k1;
            dH += s0k2;
            dI += s0k3;
            dJ += s0k4;
            dK += s0k5;

            const float s1k0 = s1 * k(0,1);
            const float s1k1 = s1 * k(1,1);
            const float s1k2 = s1 * k(2,1);
            const float s1k3 = s1 * k(3,1);
            const float s1k4 = s1 * k(4,1);
            const float s1k5 = s1 * k(5,1);
            dA += s1k5;
            dB += s1k4;
            dC += s1k3;
            dD += s1k2;
            dE += s1k1;
            dF += s1k0;
            dG += s1k1;
            dH += s1k2;
            dI += s1k3;
            dJ += s1k4;
            dK += s1k5;

            const float s2k0 = s2 * k(0,2);
            const float s2k1 = s2 * k(1,2);
            const float s2k2 = s2 * k(2,2);
            const float s2k3 = s2 * k(3,2);
            const float s2k4 = s2 * k(4,2);
            const float s2k5 = s2 * k(5,2);
            dA += s2k5;
            dB += s2k4;
            dC += s2k3;
            dD += s2k2;
            dE += s2k1;
            dF += s2k0;
            dG += s2k1;
            dH += s2k2;
            dI += s2k3;
            dJ += s2k4;
            dK += s2k5;

            const float s3k0 = s3 * k(0,3);
            const float s3k1 = s3 * k(1,3);
            const float s3k2 = s3 * k(2,3);
            const float s3k3 = s3 * k(3,3);
            const float s3k4 = s3 * k(4,3);
            const float s3k5 = s3 * k(5,3);
            dA += s3k5;
            dB += s3k4;
            dC += s3k3;
            dD += s3k2;
            dE += s3k1;
            dF += s3k0;
            dG += s3k1;
            dH += s3k2;
            dI += s3k3;
            dJ += s3k4;
            dK += s3k5;

            const float s4k0 = s4 * k(0,4);
            const float s4k1 = s4 * k(1,4);
            const float s4k2 = s4 * k(2,4);
            const float s4k3 = s4 * k(3,4);
            const float s4k4 = s4 * k(4,4);
            const float s4k5 = s4 * k(5,4);
            dA += s4k5;
            dB += s4k4;
            dC += s4k3;
            dD += s4k2;
            dE += s4k1;
            dF += s4k0;
            dG += s4k1;
            dH += s4k2;
            dI += s4k3;
            dJ += s4k4;
            dK += s4k5;

            const float s5k0 = s5 * k(0,5);
            const float s5k1 = s5 * k(1,5);
            const float s5k2 = s5 * k(2,5);
            const float s5k3 = s5 * k(3,5);
            const float s5k4 = s5 * k(4,5);
            const float s5k5 = s5 * k(5,5);
            dA += s5k5;
            dB += s5k4;
            dC += s5k3;
            dD += s5k2;
            dE += s5k1;
            dF += s5k0;
            dG += s5k1;
            dH += s5k2;
            dI += s5k3;
            dJ += s5k4;
            dK += s5k5;

            *pd = dA;  pd+=destStride;
            *pd = dB;  pd+=destStride;
            *pd = dC;  pd+=destStride;
            *pd = dD;  pd+=destStride;
            *pd = dE;  pd+=destStride;
            *pd = dF;  pd+=destStride;
            *pd = dG;  pd+=destStride;
            *pd = dH;  pd+=destStride;
            *pd = dI;  pd+=destStride;
            *pd = dJ;  pd+=destStride;
            *pd = dK;
#endif

            ++s;
            ++d;
        }
        while (--xd);

        s += srceStride - width;
        d += destStride - width;
    }
    while (--yd);

    #undef d
#endif
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


double sum_tile(uint32_t tileWidth, uint32_t tileHeight, uint32_t tileStride, double c1, double c2,
                const Float* muATile, const Float* muBTile, const Float* sigmaA2Tile, const Float* sigmaB2Tile, const Float* sigmaABTile,
                float* ssimTile, ptrdiff_t ssimStep, ptrdiff_t ssimStride) RMGR_NOEXCEPT
{
    double tileSum = 0.0f;
    for (uint32_t y=0; y<tileHeight; ++y)
    {
        const Float* muARow     = muATile     + y * tileStride;
        const Float* muBRow     = muBTile     + y * tileStride;
        const Float* sigmaA2Row = sigmaA2Tile + y * tileStride;
        const Float* sigmaB2Row = sigmaB2Tile + y * tileStride;
        const Float* sigmaABRow = sigmaABTile + y * tileStride;
        float*       ssimPtr    = ssimTile    + y * ssimStride;
        for (uint32_t x=0; x<tileWidth; ++x)
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

            if (ssimTile != NULL)
            {
                *ssimPtr = float(ssim);
                ssimPtr += ssimStep;
            }
        }
    }
    return tileSum;
}


volatile GaussianBlurFct g_gaussianBlurFct = NULL;
volatile SumTileFct      g_sumTileFct      = NULL;

}} // namespace rmgr::ssim


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
    using namespace rmgr::ssim;

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
        {
            g_gaussianBlurFct = avx::g_gaussianBlurFct;
            g_sumTileFct      = sum_tile;
        }
        else if ((edx & (1 << 25))!=0 && sse::g_gaussianBlurFct!=NULL)
        {
            g_gaussianBlurFct = sse::g_gaussianBlurFct;
            g_sumTileFct      = sum_tile;
        }
        else
        {
            g_gaussianBlurFct = gaussian_blur;
            g_sumTileFct      = sum_tile;
        }
    }
}
   

#endif // RMGR_ARCH_IS_X86_ANY


//=================================================================================================
// Main function

float rmgr::ssim::compute_ssim(uint32_t width, uint32_t height,
                               const uint8_t* imgAData, ptrdiff_t imgAStep, ptrdiff_t imgAStride,
                               const uint8_t* imgBData, ptrdiff_t imgBStep, ptrdiff_t imgBStride,
                               float* ssimMap, ptrdiff_t ssimStep, ptrdiff_t ssimStride) RMGR_NOEXCEPT
{
    GaussianBlurFct gaussianBlur = g_gaussianBlurFct;
    SumTileFct      sumTile      = g_sumTileFct;
    if (gaussianBlur == NULL)
    {
#if RMGR_ARCH_IS_X86_ANY
        init_x86();
#else
        g_gaussianBlurFct = gaussian_blur;
        g_sumTileFct      = sum_tile;
#endif
        gaussianBlur = g_gaussianBlurFct;
        sumTile      = g_sumTileFct;
    }

    const double k1 = 0.01;
    const double k2 = 0.03;
    const double L  = UINT8_MAX;
    const double c1 = (k1 * L) * (k1 * L);
    const double c2 = (k2 * L) * (k2 * L);

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

    const uint32_t tileMaxWidth   = 256;
    const uint32_t tileMaxHeight  =  64;
    const uint32_t bufferWidth    = tileMaxWidth  + 2*radius;
    const uint32_t bufferHeight   = tileMaxHeight + 4*radius;
    const size_t   bufferCapacity = ALIGN_UP(bufferWidth*bufferHeight*sizeof(Float), RMGR_SSIM_TILE_ALIGNMENT);

    double sum = 0.0;
    for (uint32_t ty=0; ty<height; ty+=tileMaxHeight)
    {
        const uint32_t tileHeight = std::min(tileMaxHeight, height-ty);
        for (uint32_t tx=0; tx<width; tx+=tileMaxWidth)
        {
            const uint32_t tileWidth           = std::min(tileMaxWidth, width-tx);
            const uint32_t tileStride          = tileWidth + 2*radius;
            const size_t   tileOffsetToTopLeft =   radius * tileStride + radius; // Offset from beginning of buffer to top-left pixel
            const size_t   blurOffsetToTopLeft = 2*radius * tileStride + radius; // Offset from beginning of buffer to top-left pixel

            RMGR_ALIGNED_VAR(RMGR_SSIM_TILE_ALIGNMENT, Float, buffers[6][bufferCapacity / sizeof(Float)]);

            Float* a = buffers[1];
            Float* b = buffers[2];
            retrieve_tile(a, tileWidth, tileHeight, tileStride, radius, tx, ty, imgAData, width, height, imgAStep, imgAStride);
            retrieve_tile(b, tileWidth, tileHeight, tileStride, radius, tx, ty, imgBData, width, height, imgBStep, imgBStride);

            Float* a2 = buffers[3];
            Float* b2 = buffers[4];
            Float* ab = buffers[5];
            multiply(a2, a, a, tileWidth+2*radius, tileHeight+2*radius, tileStride);
            multiply(b2, b, b, tileWidth+2*radius, tileHeight+2*radius, tileStride);
            multiply(ab, a, b, tileWidth+2*radius, tileHeight+2*radius, tileStride);

            Float* muA     = buffers[0] + blurOffsetToTopLeft;
            Float* muB     = buffers[1] + blurOffsetToTopLeft;
            Float* sigmaA2 = buffers[2] + blurOffsetToTopLeft;
            Float* sigmaB2 = buffers[3] + blurOffsetToTopLeft;
            Float* sigmaAB = buffers[4] + blurOffsetToTopLeft;
            gaussianBlur(muA,     tileStride, a  + tileOffsetToTopLeft, tileStride, tileWidth, tileHeight, kernel, radius);
            gaussianBlur(muB,     tileStride, b  + tileOffsetToTopLeft, tileStride, tileWidth, tileHeight, kernel, radius);
            gaussianBlur(sigmaA2, tileStride, a2 + tileOffsetToTopLeft, tileStride, tileWidth, tileHeight, kernel, radius);
            gaussianBlur(sigmaB2, tileStride, b2 + tileOffsetToTopLeft, tileStride, tileWidth, tileHeight, kernel, radius);
            gaussianBlur(sigmaAB, tileStride, ab + tileOffsetToTopLeft, tileStride, tileWidth, tileHeight, kernel, radius);

            float* ssimTile = ssimMap + tx * ssimStep + ty * ssimStride;
            sum += sum_tile(tileWidth, tileHeight, tileStride, c1, c2, muA, muB, sigmaA2, sigmaB2, sigmaAB, ssimTile, ssimStep, ssimStride);
        }
    }

    return float(sum / double(width * height));
}
