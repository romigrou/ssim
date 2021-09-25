/*
 * Copyright (c) 2021, Romain Bailly
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
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstring>
#if RMGR_SSIM_USE_OPENMP && defined(_OPENMP)
    #include <omp.h>
#endif
#if RMGR_ARCH_IS_ARM_ANY && defined(__linux__)
    #include <asm/hwcap.h>
    #include <sys/auxv.h>
#endif


#ifndef RMGR_SSIM_REPORT_ERROR
    #ifndef NDEBUG
        #define RMGR_SSIM_REPORT_ERROR(...)  fprintf(stderr, __VA_ARGS__)
    #else
        #define RMGR_SSIM_REPORT_ERROR(...)
    #endif
#endif


#ifndef RMGR_SSIM_CACHE_LINE_SIZE
    #define RMGR_SSIM_CACHE_LINE_SIZE  64
#endif

#ifndef RMGR_SSIM_TILE_ALIGNMENT
    #define RMGR_SSIM_TILE_ALIGNMENT  RMGR_SSIM_CACHE_LINE_SIZE
#endif

#define ALIGN_UP(size, alignment)  (((size) + (alignment)-1) & ~size_t((alignment)-1))


//=================================================================================================
// cpu_id()

#if RMGR_ARCH_IS_X86_ANY
    #ifdef _MSC_VER
        #include <intrin.h>
        static inline void cpu_id(int leaf, int regs[4]) RMGR_NOEXCEPT
        {
            __cpuid(regs, leaf);
        }
    #elif RMGR_COMPILER_IS_GCC_OR_CLANG
        #include <cpuid.h>
        static inline void cpu_id(int leaf, int regs[4]) RMGR_NOEXCEPT
        {
            __get_cpuid(leaf, reinterpret_cast<unsigned*>(regs), reinterpret_cast<unsigned*>(regs+1), reinterpret_cast<unsigned*>(regs+2), reinterpret_cast<unsigned*>(regs+3));
        }
    #else
        #error Unsupported compiler
    #endif
#endif


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


//=================================================================================================
// Constants

static const unsigned GAUSSIAN_RADIUS = 5;
static const Float    GAUSSIAN_SIGMA  = 1.5;

static const uint32_t TILE_MAX_WIDTH   = 256 / (sizeof(Float) / sizeof(float));
static const uint32_t TILE_MAX_HEIGHT  =  64;
static const uint32_t HORZ_MARGIN      = GAUSSIAN_RADIUS;
static const uint32_t VERT_MARGIN      = 2 * GAUSSIAN_RADIUS; // The blur routines require an extra write margin vertically (but the read margin is equal to the radius)
static const size_t   BUFFER_ALIGNMENT = RMGR_SSIM_TILE_ALIGNMENT / sizeof(Float);
static const size_t   ROW_ALIGNMENT    = BUFFER_ALIGNMENT;
static const uint32_t BUFFER_WIDTH     = TILE_MAX_WIDTH  + 2 * HORZ_MARGIN;
static const uint32_t BUFFER_HEIGHT    = TILE_MAX_HEIGHT + 2 * VERT_MARGIN;
static const size_t   BUFFER_STRIDE    = ALIGN_UP(BUFFER_WIDTH, ROW_ALIGNMENT);
static const size_t   BUFFER_CAPACITY  = ALIGN_UP(BUFFER_STRIDE * BUFFER_HEIGHT + ROW_ALIGNMENT-1, BUFFER_ALIGNMENT);


//=================================================================================================
// Allocators


void UnthreadedParams::use_default_allocator() RMGR_NOEXCEPT
{
    alloc   = default_alloc;
    dealloc = default_dealloc;
}


//=================================================================================================
// multiply()


/**
 * @brief Multiplies a tile by another tile
 */
static void multiply(Float* product, const Float* a, const Float* b, uint32_t width, uint32_t height, size_t stride, uint32_t margin) RMGR_NOEXCEPT
{
    a       -= margin * stride + margin,
    b       -= margin * stride + margin,
    product -= margin * stride + margin,
    width   += 2 * margin;
    height  += 2 * margin;
    stride  -= width;
    for (uint32_t y=0; y<height; ++y)
    {
        for (uint32_t x=0; x<width; ++x)
            *product++ = *a++ * *b++;
        a       += stride;
        b       += stride;
        product += stride;
    }
}


//=================================================================================================
// gaussian_blur()


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
            printf("%19.17e, ", kernel[x + y * size]);
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

    #define MUL_ADD(a, b, c)  ((a) + (b) * (c))
    #define REUSE_S           (RMGR_ARCH_IS_ARM_64) // 64-bit ARM has 32 registers, enough to keep everything in registers

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

#if REUSE_S
        Float s_5 = s[-5];
        Float s_4 = s[-4];
        Float s_3 = s[-3];
        Float s_2 = s[-2];
        Float s_1 = s[-1];
        Float s0  = s[ 0];
        Float s1  = s[ 1];
        Float s2  = s[ 2];
        Float s3  = s[ 3];
        Float s4  = s[ 4];
#endif
        do
        {
            // Unrolled code that exploits all the symmetries in the kernel.
            // Vertical and horizontal symmetries allow us to save a lot of computation thanks to factorization.
            // The diagonal symmetries say that k(x,y) == k(y,x), we cannot exploit that for factorizing,
            // but we can load fewer coefficients by keeping x <= y (21 coefficients instead of 36).

#if !REUSE_S
            const Float s0  = s[ 0];
            const Float s_1 = s[-1];
            const Float s1  = s[ 1];
            const Float s_2 = s[-2];
            const Float s2  = s[ 2];
            const Float s_3 = s[-3];
            const Float s3  = s[ 3];
            const Float s_4 = s[-4];
            const Float s4  = s[ 4];
            const Float s_5 = s[-5];
#endif
            const Float s5  = s[5];

            Float sum0 = s0 * k(0,0);
            Float sum1 = s0 * k(0,1);
            Float sum2 = s0 * k(0,2);
            Float sum3 = s0 * k(0,3);
            Float sum4 = s0 * k(0,4);
            Float sum5 = s0 * k(0,5);

            const Float s11 = s1 + s_1;
            sum0 = MUL_ADD(sum0, s11, k(0,1));
            sum1 = MUL_ADD(sum1, s11, k(1,1));
            sum2 = MUL_ADD(sum2, s11, k(1,2));
            sum3 = MUL_ADD(sum3, s11, k(1,3));
            sum4 = MUL_ADD(sum4, s11, k(1,4));
            sum5 = MUL_ADD(sum5, s11, k(1,5));

            const Float s22 = s2 + s_2;
            sum0 = MUL_ADD(sum0, s22, k(0,2));
            sum1 = MUL_ADD(sum1, s22, k(1,2));
            sum2 = MUL_ADD(sum2, s22, k(2,2));
            sum3 = MUL_ADD(sum3, s22, k(2,3));
            sum4 = MUL_ADD(sum4, s22, k(2,4));
            sum5 = MUL_ADD(sum5, s22, k(2,5));

            const Float s33 = s3 + s_3;
            sum0 = MUL_ADD(sum0, s33, k(0,3));
            sum1 = MUL_ADD(sum1, s33, k(1,3));
            sum2 = MUL_ADD(sum2, s33, k(2,3));
            sum3 = MUL_ADD(sum3, s33, k(3,3));
            sum4 = MUL_ADD(sum4, s33, k(3,4));
            sum5 = MUL_ADD(sum5, s33, k(3,5));

            const Float s44 = s4 + s_4;
            sum0 = MUL_ADD(sum0, s44, k(0,4));
            sum1 = MUL_ADD(sum1, s44, k(1,4));
            sum2 = MUL_ADD(sum2, s44, k(2,4));
            sum3 = MUL_ADD(sum3, s44, k(3,4));
            sum4 = MUL_ADD(sum4, s44, k(4,4));
            sum5 = MUL_ADD(sum5, s44, k(4,5));

            const Float s55 = s5 + s_5;
            sum0 = MUL_ADD(sum0, s55, k(0,5));
            sum1 = MUL_ADD(sum1, s55, k(1,5));
            sum2 = MUL_ADD(sum2, s55, k(2,5));
            sum3 = MUL_ADD(sum3, s55, k(3,5));
            sum4 = MUL_ADD(sum4, s55, k(4,5));
            sum5 = MUL_ADD(sum5, s55, k(5,5));

            Float* pd = d;
            *pd += sum5;  pd+=destStride;
            *pd += sum4;  pd+=destStride;
            *pd += sum3;  pd+=destStride;
            *pd += sum2;  pd+=destStride;
            *pd += sum1;  pd+=destStride;
            *pd += sum0;  pd+=destStride;
            *pd += sum1;  pd+=destStride;
            *pd += sum2;  pd+=destStride;
            *pd += sum3;  pd+=destStride;
            *pd += sum4;  pd+=destStride;
            *pd += sum5;

#if REUSE_S
            s_5 = s_4;
            s_4 = s_3;
            s_3 = s_2;
            s_2 = s_1;
            s_1 = s0;
            s0  = s1;
            s1  = s2;
            s2  = s3;
            s3  = s4;
            s4  = s5;
#endif

            ++s;
            ++d;
        }
        while (--xd);

        s += srceStride - width;
        d += destStride - width;
    }
    while (--yd);

    #undef k
#endif
}


//=================================================================================================
// retrieve_tile()


/**
 * @brief Retrieves a tile of an image, converting it to `Float` on the fly
 *
 * The tile is retrieved with margins, i.e. image data located around the tile. When the tile
 * is located on a border of the image, data is simply duplicated to fill the tile as needed.
 *
 * @param [out] tile        Pointer to the tile's top-left pixel (i.e. exclusive of margins)
 * @param [in]  tileWidth   The tile's width,  in pixels
 * @param [in]  tileHeight  The tile's height, in pixels
 * @param [in]  tileStride  Distance between a tile row and the row below it, in number of `Float`s
 * @param [in]  margin      The size of the margin around the tile, in pixels
 * @param [in]  x           Horizontal coordinate of the tile's top-left corner within the image
 * @param [in]  y           Vertical   coordinate of the tile's top-left corner within the image
 * @param [in]  imgData     Pointer to the top-left pixel of the image
 * @param [in]  imgWidth    Image's width,  in pixels
 * @param [in]  imgHeight   Image's height, in pixels
 * @param [in]  imgStep     Distance between a pixel of the image and the one immediatelt to its right.
 * @param [in]  imgStride   Distance between a pixel of the image and the one immediately below it.
 */
static void retrieve_tile(Float* tile, uint32_t tileWidth, uint32_t tileHeight, size_t tileStride, uint32_t margin, uint32_t x, uint32_t y,
                          const uint8_t* imgData, uint32_t imgWidth, uint32_t imgHeight, ptrdiff_t imgStep, ptrdiff_t imgStride) RMGR_NOEXCEPT
{
    assert(tileStride >= tileWidth + 2*margin);
    assert(0<=x && x<imgWidth);
    assert(0<=y && y<imgHeight);

    // Point to start of buffer rather than top-left corner
    tile -= margin * tileStride + margin;

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


//=================================================================================================
// sum_tile()


static double sum_tile(uint32_t tileWidth, uint32_t tileHeight, uint32_t tileStride, Float c1, Float c2,
                       const Float* muATile, const Float* muBTile, const Float* sigmaA2Tile, const Float* sigmaB2Tile, const Float* sigmaABTile,
                       float* ssimTile, ptrdiff_t ssimStep, ptrdiff_t ssimStride) RMGR_NOEXCEPT
{
    double tileSum = 0.0; // The sum is always done on a double to increase precision

    for (uint32_t y=0; y<tileHeight; ++y)
    {
        const Float* muARow     = muATile     + y * tileStride;
        const Float* muBRow     = muBTile     + y * tileStride;
        const Float* sigmaA2Row = sigmaA2Tile + y * tileStride;
        const Float* sigmaB2Row = sigmaB2Tile + y * tileStride;
        const Float* sigmaABRow = sigmaABTile + y * tileStride;
        float*       ssimPtr    = ssimTile    + y * ssimStride;

        int32_t x = tileWidth;
        if ((x -= 4) >= 0)
        {
            double rowSum1 = 0.0;
            double rowSum2 = 0.0;
            double rowSum3 = 0.0;
            double rowSum4 = 0.0;
            do
            {
                const Float muA_1     = *muARow++;
                const Float muB_1     = *muBRow++;
                const Float muA_2     = *muARow++;
                const Float muB_2     = *muBRow++;
                const Float muA_3     = *muARow++;
                const Float muB_3     = *muBRow++;
                const Float muA_4     = *muARow++;
                const Float muB_4     = *muBRow++;
                const Float muA2_1    = muA_1 * muA_1;
                const Float muB2_1    = muB_1 * muB_1;
                const Float muAB_1    = muA_1 * muB_1;
                const Float muA2_2    = muA_2 * muA_2;
                const Float muB2_2    = muB_2 * muB_2;
                const Float muAB_2    = muA_2 * muB_2;
                const Float muA2_3    = muA_3 * muA_3;
                const Float muB2_3    = muB_3 * muB_3;
                const Float muAB_3    = muA_3 * muB_3;
                const Float muA2_4    = muA_4 * muA_4;
                const Float muB2_4    = muB_4 * muB_4;
                const Float muAB_4    = muA_4 * muB_4;
                const Float sigmaA2_1 = *sigmaA2Row++ - muA2_1;
                const Float sigmaB2_1 = *sigmaB2Row++ - muB2_1;
                const Float sigmaAB_1 = *sigmaABRow++ - muAB_1;
                const Float sigmaA2_2 = *sigmaA2Row++ - muA2_2;
                const Float sigmaB2_2 = *sigmaB2Row++ - muB2_2;
                const Float sigmaAB_2 = *sigmaABRow++ - muAB_2;
                const Float sigmaA2_3 = *sigmaA2Row++ - muA2_3;
                const Float sigmaB2_3 = *sigmaB2Row++ - muB2_3;
                const Float sigmaAB_3 = *sigmaABRow++ - muAB_3;
                const Float sigmaA2_4 = *sigmaA2Row++ - muA2_4;
                const Float sigmaB2_4 = *sigmaB2Row++ - muB2_4;
                const Float sigmaAB_4 = *sigmaABRow++ - muAB_4;

                const Float numerator1   = (2 * muAB_1 + c1) * (2 * sigmaAB_1 + c2);
                const Float numerator2   = (2 * muAB_2 + c1) * (2 * sigmaAB_2 + c2);
                const Float numerator3   = (2 * muAB_3 + c1) * (2 * sigmaAB_3 + c2);
                const Float numerator4   = (2 * muAB_4 + c1) * (2 * sigmaAB_4 + c2);
                const Float denominator1 = (muA2_1 + muB2_1 + c1) * (sigmaA2_1 + sigmaB2_1 + c2);
                const Float denominator2 = (muA2_2 + muB2_2 + c1) * (sigmaA2_2 + sigmaB2_2 + c2);
                const Float denominator3 = (muA2_3 + muB2_3 + c1) * (sigmaA2_3 + sigmaB2_3 + c2);
                const Float denominator4 = (muA2_4 + muB2_4 + c1) * (sigmaA2_4 + sigmaB2_4 + c2);

                const Float ssim1 = numerator1 / denominator1;
                const Float ssim2 = numerator2 / denominator2;
                const Float ssim3 = numerator3 / denominator3;
                const Float ssim4 = numerator4 / denominator4;

                rowSum1 += ssim1;
                rowSum2 += ssim2;
                rowSum3 += ssim3;
                rowSum4 += ssim4;

                if (ssimTile != NULL)
                {
                    *ssimPtr = float(ssim1);  ssimPtr += ssimStep;
                    *ssimPtr = float(ssim2);  ssimPtr += ssimStep;
                    *ssimPtr = float(ssim3);  ssimPtr += ssimStep;
                    *ssimPtr = float(ssim4);  ssimPtr += ssimStep;
                }
            }
            while ((x -= 4) >= 0);
            tileSum += (rowSum1 + rowSum2) + (rowSum3 + rowSum4);
        }
        x += 4;

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

            if (ssimTile != NULL)
            {
                *ssimPtr = float(ssim);
                ssimPtr += ssimStep;
            }
        }
    }

    return tileSum;
}


//=================================================================================================
// process_tile()


struct GlobalParams
{
    // User-specified parameters
    uint32_t        width;
    uint32_t        height;
    ImgParams       imgA;
    ImgParams       imgB;
    float*          ssimMap;
    ptrdiff_t       ssimStep;
    ptrdiff_t       ssimStride;

    // Other parameters
    uint32_t        tileMaxWidth;
    uint32_t        tileMaxHeight;
    uint32_t        horzMargin;
    uint32_t        vertMargin;
    uint32_t        rowAlignment;
    size_t          bufferCapacity;
    const Float*    gaussianKernel;
    uint32_t        gaussianRadius;
    Float           c1;
    Float           c2;
    MultiplyFct     multiply;
    GaussianBlurFct gaussianBlur;
    SumTileFct      sumTile;
};


struct TileParams
{
    Float*   buffers[6];
    uint32_t tileX;
    uint32_t tileY;
};


static double process_tile(const TileParams& tp, const GlobalParams& gp) RMGR_NOEXCEPT
{
    const uint32_t tileWidth  = std::min(gp.tileMaxWidth,  gp.width  - tp.tileX);
    const uint32_t tileHeight = std::min(gp.tileMaxHeight, gp.height - tp.tileY);
    const uint32_t tileStride = uint32_t(ALIGN_UP(tileWidth + 2*gp.horzMargin, gp.rowAlignment));

    // Compute offset between start of buffer and top-left pixel
    const size_t offsetToTopLeft = gp.vertMargin * tileStride + ALIGN_UP(gp.horzMargin, gp.rowAlignment);
    assert(offsetToTopLeft % gp.rowAlignment == 0);
    assert(offsetToTopLeft + (tileStride * (tileHeight + gp.vertMargin)) - gp.horzMargin <= gp.bufferCapacity);

    Float* a = tp.buffers[1] + offsetToTopLeft;
    Float* b = tp.buffers[2] + offsetToTopLeft;
    retrieve_tile(a, tileWidth, tileHeight, tileStride, gp.gaussianRadius, tp.tileX, tp.tileY, gp.imgA.topLeft, gp.width, gp.height, gp.imgA.step, gp.imgA.stride);
    retrieve_tile(b, tileWidth, tileHeight, tileStride, gp.gaussianRadius, tp.tileX, tp.tileY, gp.imgB.topLeft, gp.width, gp.height, gp.imgB.step, gp.imgB.stride);

    Float* a2 = tp.buffers[3] + offsetToTopLeft;
    Float* b2 = tp.buffers[4] + offsetToTopLeft;
    Float* ab = tp.buffers[5] + offsetToTopLeft;
    gp.multiply(a2, a, a, tileWidth, tileHeight, tileStride, gp.gaussianRadius);
    gp.multiply(b2, b, b, tileWidth, tileHeight, tileStride, gp.gaussianRadius);
    gp.multiply(ab, a, b, tileWidth, tileHeight, tileStride, gp.gaussianRadius);

    Float* muA     = tp.buffers[0] + offsetToTopLeft;
    Float* muB     = tp.buffers[1] + offsetToTopLeft;
    Float* sigmaA2 = tp.buffers[2] + offsetToTopLeft;
    Float* sigmaB2 = tp.buffers[3] + offsetToTopLeft;
    Float* sigmaAB = tp.buffers[4] + offsetToTopLeft;
    gp.gaussianBlur(muA,     tileStride, a,  tileStride, tileWidth, tileHeight, gp.gaussianKernel, gp.gaussianRadius);
    gp.gaussianBlur(muB,     tileStride, b,  tileStride, tileWidth, tileHeight, gp.gaussianKernel, gp.gaussianRadius);
    gp.gaussianBlur(sigmaA2, tileStride, a2, tileStride, tileWidth, tileHeight, gp.gaussianKernel, gp.gaussianRadius);
    gp.gaussianBlur(sigmaB2, tileStride, b2, tileStride, tileWidth, tileHeight, gp.gaussianKernel, gp.gaussianRadius);
    gp.gaussianBlur(sigmaAB, tileStride, ab, tileStride, tileWidth, tileHeight, gp.gaussianKernel, gp.gaussianRadius);

    float* ssimTile = gp.ssimMap + (tp.tileX * gp.ssimStep) + (tp.tileY * gp.ssimStride);
    return gp.sumTile(tileWidth, tileHeight, tileStride, gp.c1, gp.c2, muA, muB, sigmaA2, sigmaB2, sigmaAB, ssimTile, gp.ssimStep, gp.ssimStride);
};


double process_tile_on_stack(uint32_t tileX, uint32_t tileY, const GlobalParams& gp)
{
    RMGR_ALIGNED_VAR(RMGR_SSIM_TILE_ALIGNMENT, Float, buffers[6][BUFFER_CAPACITY]);
    const TileParams tp = {{buffers[0],buffers[1],buffers[2],buffers[3],buffers[4],buffers[5]}, tileX, tileY};
    return process_tile(tp, gp);
}


//=================================================================================================
// init_impl()


volatile MultiplyFct     g_multiplyFct     = NULL;
volatile GaussianBlurFct g_gaussianBlurFct = NULL;
volatile SumTileFct      g_sumTileFct      = NULL;


/**
 * @brief Selects the most suitable implementation based on what's actually supported by the hardware
 *
 * @returns A bit mask of all the supported implementations
 */
unsigned select_impl(Implementation desiredImpl) RMGR_NOEXCEPT
{
    unsigned supportedImpls = (1 << IMPL_AUTO) | (1 << IMPL_GENERIC);

    MultiplyFct     multiplyFct     = NULL;
    GaussianBlurFct gaussianBlurFct = NULL;
    SumTileFct      sumTileFct      = NULL;

#if RMGR_ARCH_IS_X86_ANY
    // Detect machine's features
    int regs[4] = {};
    cpu_id(0, regs);
    const uint32_t maxLeaf = regs[0];
    if (maxLeaf >= 1)
    {
        cpu_id(1, regs);
        const uint32_t ecx = regs[2];
        const uint32_t edx = regs[3];

        supportedImpls |= (((edx >> 25) & 1) && sse::g_gaussianBlurFct!=NULL)  ? (1 << IMPL_SSE)  : 0;
        supportedImpls |= (((edx >> 26) & 1) && sse2::g_gaussianBlurFct!=NULL) ? (1 << IMPL_SSE2) : 0;
        supportedImpls |= (((ecx >> 28) & 1) && avx::g_gaussianBlurFct!=NULL)  ? (1 << IMPL_AVX)  : 0;
        supportedImpls |= (((ecx >> 12) & 1) && fma::g_gaussianBlurFct!=NULL)  ? (1 << IMPL_FMA)  : 0;

    }
    if (maxLeaf >= 7)
    {
        cpu_id(7, regs);
        const uint32_t ebx = regs[1];

        supportedImpls |= (((ebx >> 16) & 1) && avx512::g_gaussianBlurFct!=NULL) ? (1 << IMPL_AVX512) : 0;
    }

    // Select the most suitable implementation
    if ((supportedImpls & (1 << IMPL_SSE)) && (desiredImpl==IMPL_AUTO || desiredImpl==IMPL_SSE))
    {
        multiplyFct     = sse::g_multiplyFct;
        gaussianBlurFct = sse::g_gaussianBlurFct;
    }
    if ((supportedImpls & (1 << IMPL_SSE2)) && (desiredImpl==IMPL_AUTO || desiredImpl==IMPL_SSE2))
    {
        multiplyFct     = sse2::g_multiplyFct;
        gaussianBlurFct = sse2::g_gaussianBlurFct;
        sumTileFct      = sse2::g_sumTileFct;
    }
    if ((supportedImpls & (1 << IMPL_AVX)) && (desiredImpl==IMPL_AUTO || desiredImpl==IMPL_AVX))
    {
        multiplyFct     = avx::g_multiplyFct;
        gaussianBlurFct = avx::g_gaussianBlurFct;
        sumTileFct      = avx::g_sumTileFct;
    }
    if ((supportedImpls & (1 << IMPL_FMA)) && (desiredImpl==IMPL_AUTO || desiredImpl==IMPL_FMA))
    {
        multiplyFct     = fma::g_multiplyFct;
        gaussianBlurFct = fma::g_gaussianBlurFct;
        sumTileFct      = fma::g_sumTileFct;
    }
    if ((supportedImpls & (1 << IMPL_AVX512)) && (desiredImpl==IMPL_AUTO || desiredImpl==IMPL_AVX512))
    {
        multiplyFct     = avx512::g_multiplyFct;
        gaussianBlurFct = avx512::g_gaussianBlurFct;
        sumTileFct      = avx512::g_sumTileFct;
    }

#elif RMGR_ARCH_IS_ARM_ANY
    #ifdef __linux__
        const long hwcap = getauxval(AT_HWCAP);
        #if RMGR_ARCH_IS_ARM_64
            supportedImpls |= (hwcap & HWCAP_ASIMD) ? (1 << IMPL_NEON) : 0;
        #else
            supportedImpls |= (hwcap & HWCAP_NEON)  ? (1 << IMPL_NEON) : 0;
        #endif
    #endif

    if ((supportedImpls & (1 << IMPL_NEON)) && (desiredImpl==IMPL_AUTO || desiredImpl==IMPL_NEON))
    {
        multiplyFct     = neon::g_multiplyFct;
        gaussianBlurFct = neon::g_gaussianBlurFct;
    }

#endif // RMGR_ARCH_IS_X86_ANY

    // Fall back to generic implementation if needed
    g_multiplyFct     = (multiplyFct     != NULL) ? multiplyFct     : multiply;
    g_gaussianBlurFct = (gaussianBlurFct != NULL) ? gaussianBlurFct : gaussian_blur;
    g_sumTileFct      = (sumTileFct      != NULL) ? sumTileFct      : sum_tile;

    return supportedImpls;
}


//=================================================================================================
// Threaded processing

struct ThreadParams
{
    double              value;
    const GlobalParams* globalParams;
    unsigned            tileHorzCount;
    TileParams          tileParams;
};


static void process_tile_in_thread(void* arg, unsigned tileNum) RMGR_NOEXCEPT
{
    ThreadParams& p = *static_cast<ThreadParams*>(arg);
    p.tileParams.tileX = (tileNum % p.tileHorzCount) * p.globalParams->tileMaxWidth;
    p.tileParams.tileY = (tileNum / p.tileHorzCount) * p.globalParams->tileMaxHeight;
    p.value += process_tile(p.tileParams, *p.globalParams);
}


static void process_tile_on_stack_in_thread(void* arg, unsigned tileNum) RMGR_NOEXCEPT
{
    ThreadParams& p = *static_cast<ThreadParams*>(arg);
    unsigned tileX = (tileNum % p.tileHorzCount) * p.globalParams->tileMaxWidth;
    unsigned tileY = (tileNum / p.tileHorzCount) * p.globalParams->tileMaxHeight;
    p.value += process_tile_on_stack(tileX, tileY, *p.globalParams);
}


//=================================================================================================
// Main function


float compute_ssim(const Params& params) RMGR_NOEXCEPT
{
    MultiplyFct     multiplyFct     = g_multiplyFct;
    GaussianBlurFct gaussianBlurFct = g_gaussianBlurFct;
    SumTileFct      sumTileFct      = g_sumTileFct;
    if (multiplyFct==NULL || gaussianBlurFct==NULL || multiplyFct==NULL)
    {
        select_impl(IMPL_AUTO);

        multiplyFct     = g_multiplyFct;
        gaussianBlurFct = g_gaussianBlurFct;
        sumTileFct      = g_sumTileFct;
    }

    // SIMD versions of sum_tile() only support ssimStep==1 and when not using double
#if RMGR_SSIM_USE_DOUBLE
    if (ssimMap != NULL)
        sumTileFct = sum_tile;
#else
    if (params.ssimMap!=NULL && params.ssimStep!=1)
        sumTileFct = sum_tile;
#endif

    const double k1 = 0.01;
    const double k2 = 0.03;
    const double L  = UINT8_MAX;
    const Float  c1 = Float((k1 * L) * (k1 * L));
    const Float  c2 = Float((k2 * L) * (k2 * L));

    if (params.imgA.topLeft==NULL || params.imgB.topLeft==NULL)
    {
        RMGR_SSIM_REPORT_ERROR("Invalid parameter: imgA.topLeft or imgB.topLeft is NULL\n");
        return -EINVAL;
    }

    if (params.threadPool != NULL && params.threadCount == 0u)
    {
        RMGR_SSIM_REPORT_ERROR("Invalid parameter: threadCount cannot be 0 if threadPool is not NULL\n");
        return -EINVAL;
    }

    float*    ssimMap    = params.ssimMap;
    ptrdiff_t ssimStep   = params.ssimStep;
    ptrdiff_t ssimStride = params.ssimStride;
    if (ssimMap == NULL)
    {
        ssimStep   = 0;
        ssimStride = 0;
    }

    // Parameters of the gaussian blur
    Float kernelBuffer[(2*GAUSSIAN_RADIUS+1) * (2*GAUSSIAN_RADIUS+1)];
    precompute_gaussian_kernel(kernelBuffer, GAUSSIAN_RADIUS, GAUSSIAN_SIGMA);

    Float const* const gaussianKernel = kernelBuffer + GAUSSIAN_RADIUS * (2*GAUSSIAN_RADIUS+1) + GAUSSIAN_RADIUS; // Center of the kernel

    // Prepare global parameters
    const uint32_t width  = params.width;
    const uint32_t height = params.height;
    const GlobalParams gp =
    {
        // User-specified parameters
        width,
        height,
        params.imgA,
        params.imgB,
        ssimMap,
        ssimStep,
        ssimStride,

        // Other parameters
        TILE_MAX_WIDTH,
        TILE_MAX_HEIGHT,
        HORZ_MARGIN,
        VERT_MARGIN,
        ROW_ALIGNMENT,
        BUFFER_CAPACITY,
        gaussianKernel,
        GAUSSIAN_RADIUS,
        c1,
        c2,
        multiplyFct,
        gaussianBlurFct,
        sumTileFct
    };

    const unsigned maxThreadCount    = sizeof(void*) * CHAR_BIT; // Good heuristic
    const unsigned tileHorzCount     = (width  + TILE_MAX_WIDTH  - 1) / TILE_MAX_WIDTH;
    const unsigned tileVertCount     = (height + TILE_MAX_HEIGHT - 1) / TILE_MAX_HEIGHT;
    const unsigned tileCount         = tileHorzCount * tileVertCount;
    const unsigned actualThreadCount = (params.threadPool != NULL) ? std::min(params.threadCount, maxThreadCount) : 1;

    // Prepare thread params
    ThreadParams threadParams[maxThreadCount] = {};
    void*        threadArgs[maxThreadCount]   = {};
    if (params.threadPool != NULL)
    {
        for (unsigned threadNum=0; threadNum < actualThreadCount; ++threadNum)
        {
            ThreadParams& p = threadParams[threadNum];
            p.tileHorzCount = tileHorzCount;
            p.globalParams  = &gp;
            threadArgs[threadNum] = &threadParams[threadNum];
        }
    }

    // Process all tiles
    double sum = 0.0;
    int threadPoolResult = 0;
    if (params.alloc != NULL) // Use heap-allocated buffers
    {
        Float* heapBuffer = static_cast<Float*>(params.alloc(6 * BUFFER_CAPACITY * actualThreadCount * sizeof(Float), RMGR_SSIM_TILE_ALIGNMENT));
        if (heapBuffer == NULL)
            return -ENOMEM;

        if (params.threadPool != NULL)
        {
            for (unsigned threadNum=0; threadNum < actualThreadCount; ++threadNum)
            {
                for (int i=0; i<6; ++i)
                    threadParams[threadNum].tileParams.buffers[i] = heapBuffer + (6*threadNum + i) * BUFFER_CAPACITY;
            }
            threadPoolResult = params.threadPool(params.threadPoolContext, process_tile_in_thread, threadArgs, actualThreadCount, tileCount);
        }
        else
        {
            for (uint32_t tileY=0; tileY<height; tileY+=TILE_MAX_HEIGHT)
            {
                for (uint32_t tileX=0; tileX<width; tileX+=TILE_MAX_WIDTH)
                {
                    #define b(n) (heapBuffer + BUFFER_CAPACITY * n)
                    const TileParams tp = {{b(0),b(1),b(2),b(3),b(4),b(5)}, tileX, tileY};
                    sum += process_tile(tp, gp);
                    #undef b
                }
            }
        }
        params.dealloc(heapBuffer);
    }
    else                      // Use on-stack buffers
    {
        if (params.threadPool != NULL)
            threadPoolResult = params.threadPool(params.threadPoolContext, process_tile_on_stack_in_thread, threadArgs, actualThreadCount, tileCount);
        else
        {
            for (uint32_t tileY=0; tileY<height; tileY+=TILE_MAX_HEIGHT)
                for (uint32_t tileX=0; tileX<width; tileX+=TILE_MAX_WIDTH)
                    sum += process_tile_on_stack(tileX, tileY, gp);
        }
    }

    // In case of threaded processing, perform the final sum
    if (params.threadPool != NULL)
    {
        if (threadPoolResult != 0)
            return -ECHILD;
        for (unsigned threadNum=0; threadNum<actualThreadCount; ++threadNum)
            sum += threadParams[threadNum].value;
    }

    return float(sum / double(width * height));
}


}} // namespace rmgr::ssim
