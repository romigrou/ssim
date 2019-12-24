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


class Image
{
public:
    Image()  RMGR_NOEXCEPT: m_buffer(NULL) {}
    ~Image() RMGR_NOEXCEPT {release();}

    void release() RMGR_NOEXCEPT
    {
        dealloc(m_buffer);
        m_buffer = NULL;
    }


    int init(uint32_t width, uint32_t height, uint32_t margin=0)
    {
        release();

        const uint32_t actualWidth  = width  + 2 * margin;
        const uint32_t actualHeight = height + 2 * margin;
        const size_t   bufStride    = align_up(actualWidth*sizeof(Float), CACHE_ALIGNMENT) / sizeof(Float);
        const size_t   bufSize      = bufStride * actualHeight * sizeof(Float);
        m_buffer = static_cast<Float*>(alloc(bufSize, CACHE_ALIGNMENT));
        if (m_buffer == NULL)
        {
            RMGR_SSIM_REPORT_ERROR("Allocation failure\n");
            return -ENOMEM;
        }
        m_width  = width;
        m_height = height;
        m_stride = bufStride;
        m_margin = margin;

        return 0;
    }


    int init(const uint8_t* data, uint32_t width, uint32_t height, ptrdiff_t step, ptrdiff_t stride, uint32_t margin=0) RMGR_NOEXCEPT
    {
        release();

        const uint32_t actualWidth  = width  + 2 * margin;
        const uint32_t actualHeight = height + 2 * margin;
        const size_t   bufStride    = align_up(actualWidth*sizeof(Float), CACHE_ALIGNMENT) / sizeof(Float);
        const size_t   bufSize      = bufStride * actualHeight * sizeof(Float);
        m_buffer = static_cast<Float*>(alloc(bufSize, CACHE_ALIGNMENT));
        if (m_buffer == NULL)
        {
            RMGR_SSIM_REPORT_ERROR("Allocation failure\n");
            return -ENOMEM;
        }
        m_width  = width;
        m_height = height;
        m_stride = bufStride;
        m_margin = margin;

        const uint8_t* s = data;
        Float*         d = m_buffer + (margin * m_stride);
        const size_t   sStride = stride - step * width;
        const size_t   dStride = bufStride - actualWidth;
        for (uint32_t y=0; y<height; ++y)
        {
            // Left margin
            for (uint32_t x=0; x<margin; ++x)
                *d++ = Float(*s);

            // Actual data
            for (uint32_t x=0; x<width; ++x)
            {
                *d++ = Float(*s);
                s += step;
            }

            // Right margin
            const Float right = d[-1];
            for (uint32_t x=0; x<margin; ++x)
                *d++ = right;

            s += sStride;
            d += dStride;
        }

        // Top and bottom margins
        Float* firstRow = m_buffer + (margin * bufStride);
        Float* lastRow  = d - bufStride;
        for (uint32_t y=1; y<=margin; ++y)
        {
            memcpy(firstRow - y*bufStride, firstRow, actualWidth*sizeof(Float));
            memcpy(lastRow  + y*bufStride, lastRow,  actualWidth*sizeof(Float));
        }

        return 0;
    }

    const Float* operator[](int32_t y) const RMGR_NOEXCEPT {assert(-int32_t(m_margin)<=y && y<int32_t(m_height+m_margin)); return m_buffer + (int32_t(y + m_margin) * ptrdiff_t(m_stride)) + m_margin;}
    Float*       operator[](int32_t y)       RMGR_NOEXCEPT {assert(-int32_t(m_margin)<=y && y<int32_t(m_height+m_margin)); return m_buffer + (int32_t(y + m_margin) * ptrdiff_t(m_stride)) + m_margin;}

    const Float* buffer() const RMGR_NOEXCEPT {return m_buffer;}
    Float*       buffer()       RMGR_NOEXCEPT {return m_buffer;}
    const Float* data()   const RMGR_NOEXCEPT {return m_buffer + (m_margin * m_stride) + m_margin;}
    Float*       data()         RMGR_NOEXCEPT {return m_buffer + (m_margin * m_stride) + m_margin;}
    uint32_t     width()  const RMGR_NOEXCEPT {return m_width;}
    uint32_t     height() const RMGR_NOEXCEPT {return m_height;}
    size_t       stride() const RMGR_NOEXCEPT {return m_stride;}
    uint32_t     margin() const RMGR_NOEXCEPT {return m_margin;}

private:
    Float*   m_buffer; ///< Pointer to the buffer holding data
    uint32_t m_width;  ///< Image's width,  in pixels
    uint32_t m_height; ///< Image's height, in pixels
    size_t   m_stride; ///< Buffer's stride in number of floats
    uint32_t m_margin;
   
};


/**
 * @brief Multiplies an image by another one
 */
static int multiply(Image& product, const Image& a, const Image& b) RMGR_NOEXCEPT
{
    assert(a.width()  == b.width());
    assert(a.height() == b.height());
    assert(a.margin() == b.margin());

    if (&product != &a && &product != &b)
    {
        int err = product.init(a.width(), a.height(), a.margin());
        if (err != 0)
            return err;
    }

    const uint32_t w  = a.width()  + 2 * a.margin();
    const uint32_t h  = a.height() + 2 * a.margin();
    const Float*   pa = a.buffer();
    const Float*   pb = b.buffer();
    Float*         pp = product.buffer();
    for (uint32_t y=0; y<h; ++y)
    {
        for (uint32_t x=0; x<w; ++x)
            *pp++ = *pa++ * *pb++;
        pa += a.stride() - w;
        pb += b.stride() - w;
        pp += product.stride() - w;
    }

    return 0;
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


static int gaussian_blur(Image& dest, const Image& srce, const Float kernel[], int radius) RMGR_NOEXCEPT
{
    assert(unsigned(radius) <= srce.margin());

    int err = dest.init(srce.width(), srce.height());
    if (err != 0)
        return err;

    const int32_t width  = srce.width();
    const int32_t height = srce.height();
    for (int32_t yd=0; yd<height; ++yd)
    {
        for (int32_t xd=0; xd<width; ++xd)
        {
            Float val = 0;
            const Float* k = kernel;
            for (int32_t ys=yd-radius; ys<=yd+radius; ++ys)
            {
                for (int32_t xs=xd-radius; xs<=xd+radius; ++xs)
                    val += *k++ * srce[ys][xs];
            }
            dest[yd][xd] = val;
        }
    }

    return 0;
}


float compute_ssim(uint32_t width, uint32_t height,
                   const uint8_t* img1Data, ptrdiff_t img1Step, ptrdiff_t img1Stride,
                   const uint8_t* img2Data, ptrdiff_t img2Step, ptrdiff_t img2Stride,
                   float* ssimMap, ptrdiff_t ssimStep, ptrdiff_t ssimStride) RMGR_NOEXCEPT
{
    const Float k1 = Float(0.01);
    const Float k2 = Float(0.03);
    const Float L  = UINT8_MAX;
    const Float c1 = (k1 * L) * (k1 * L);
    const Float c2 = (k2 * L) * (k2 * L);

    if (img1Data==NULL || img2Data==NULL)
    {
        RMGR_SSIM_REPORT_ERROR("Invalid parameter: img1Data or img2Data is NULL\n");
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

    int err;
    #define CHECK(exp) if ((err = exp) != 0) return float(err)

    // Convert images to float
    Image a, b;
    if ((err = a.init(img1Data, width, height, img1Step, img1Stride, radius)) != 0)
        return float(err);
    if ((err = b.init(img2Data, width, height, img2Step, img2Stride, radius)) != 0)
        return float(err);

    // Multiply them
    Image a2, b2, ab;
    CHECK(multiply(a2, a, a));
    CHECK(multiply(b2, b, b));
    CHECK(multiply(ab, a, b));

    // Apply Gaussian blur to all above images
    Float kernel[(2*radius+1) * (2*radius+1)];
    precompute_gaussian_kernel(kernel, radius, sigma);
    Image muAImg, muBImg, sigmaA2Img, sigmaB2Img, sigmaABImg;
    CHECK(gaussian_blur(muAImg,     a,  kernel, radius));
    CHECK(gaussian_blur(muBImg,     b,  kernel, radius));
    CHECK(gaussian_blur(sigmaA2Img, a2, kernel, radius));
    CHECK(gaussian_blur(sigmaB2Img, b2, kernel, radius));
    CHECK(gaussian_blur(sigmaABImg, ab, kernel, radius));

    double sum = 0.0f;
    for (uint32_t y=0; y<height; ++y)
    {
        for (uint32_t x=0; x<width; ++x)
        {
            const double muA     = muAImg[y][x];
            const double muB     = muBImg[y][x];
            const double muA2    = muA * muA;
            const double muB2    = muB * muB;
            const double muAB    = muA * muB;
            const double sigmaA2 = sigmaA2Img[y][x] - muA2;
            const double sigmaB2 = sigmaB2Img[y][x] - muB2;
            const double sigmaAB = sigmaABImg[y][x] - muAB;

            const double numerator1   = 2 * muAB    + c1;
            const double numerator2   = 2 * sigmaAB + c2;
            const double denominator1 = muA2 + muB2 + c1;
            const double denominator2 = sigmaA2 + sigmaB2 + c2;

            const double numerator   = numerator1 * numerator2;
            const double denominator = denominator1 * denominator2;

            const double ssim = numerator / denominator;
            sum += ssim;

            if (ssimMap != NULL)
            {
                *ssimMap = Float(ssim);
                ssimMap += ssimStep;
            }
        }
        ssimMap += ssimStride - int32_t(width) * ssimStep;
    }

    return float(sum / double(width * height));
}


}} // namespace rmgr::ssim
