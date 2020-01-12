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

#include "ssim_naive.h"
#include <rmgr/ssim.h>
#include "../src/ssim_internal.h"
#include <gtest/gtest.h>


#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
RMGR_WARNING_PUSH()
RMGR_WARNING_MSVC_DISABLE(4100) // unreferenced formal parameter
RMGR_WARNING_MSVC_DISABLE(4244) // 'conversion' conversion from 'type1' to 'type2', possible loss of data
#include "stb_image.h"
RMGR_WARNING_POP()

#if RMGR_COMPILER_IS_MSVC
    #define snprintf  _snprintf_s
#endif


const unsigned IMPL_COUNT = 6;


static const char  g_defaultImagesDir[] = {RMGR_SSIM_TESTS_IMAGES_DIR};
static const char* g_imagesDir = g_defaultImagesDir;


/// Floating-point type for computing reference SSIMs. The more precise, the better.
typedef long double RefFloat;


struct RefSsim
{
    RefFloat  ssim;
    RefFloat* map;

    ~RefSsim() {delete[] map;}
};


static RefFloat g_largestGlobalError[IMPL_COUNT] = {};
static RefFloat g_largestPixelError[IMPL_COUNT]  = {};

#if RMGR_SSIM_USE_DOUBLE
    static const double GLOBAL_TOLERANCE = 5e-7f;
    static const double PIXEL_TOLERANCE  = 1e-5f;
#else
    static const double GLOBAL_TOLERANCE = 2e-6f;
    static const double PIXEL_TOLERANCE  = 1e-3f;
#endif
    

#ifdef _WIN32
    #define NOMINMAX
    #include <windows.h>

    typedef rmgr::ssim::uint64_t Ticks;

    static inline Ticks get_ticks() RMGR_NOEXCEPT
    {
        LARGE_INTEGER counter;
        QueryPerformanceCounter(&counter);
        return counter.QuadPart;
    }

    static Ticks ticks_to_us(Ticks counter) RMGR_NOEXCEPT
    {
        LARGE_INTEGER freq;
        QueryPerformanceFrequency(&freq);
        return (counter * 1000000) / freq.QuadPart;
    }
#else
    #include <time.h>

    typedef rmgr::ssim::uint64_t Ticks;

    static inline Ticks get_ticks() RMGR_NOEXCEPT
    {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return Ticks(ts.tv_sec) * 1000000000u + ts.tv_nsec;
    }

    static Ticks ticks_to_us(Ticks counter) RMGR_NOEXCEPT
    {
        return counter / 1000u;
    }
#endif


struct PerfInfo
{
    Ticks         ticks;
    unsigned long pixelCount;
};


static PerfInfo g_perfInfo[IMPL_COUNT][8] = {};


extern "C" int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    if (argc > 1)
        g_imagesDir = argv[1];

    const int result = RUN_ALL_TESTS();

    static char const* const implNames[IMPL_COUNT] = {"Auto", "Generic", "SSE", "AVX", "FMA", "Neon"};

    printf("\n"
           "         ||=================|=================|\n"
           "         ||   Global Error  | Per-Pixel Error |\n"
           "|========||=================|=================|\n");
    for (unsigned impl=0; impl<IMPL_COUNT; ++impl)
    {
        if (g_largestGlobalError[impl] != 0)
            printf("|%-7s || %11.9Le | %11.9Le |\n", implNames[impl], static_cast<long double>(g_largestGlobalError[impl]), static_cast<long double>(g_largestPixelError[impl]));
    }
    printf("|========||=================|=================|\n\n");

    printf("\n"
           "         ||=====================|=====================|=====================|=====================|\n"
           "         ||     Without map     |      With map       |   OpenMp w/o map    |    OpenMp w/ map    |\n"
           "|========||=====================|=====================|=====================|=====================|");
    for (unsigned impl=0; impl<IMPL_COUNT; ++impl)
    {
        const PerfInfo* info = g_perfInfo[impl];

        bool empty = true;
        for (unsigned i=0; i<8u; ++i)
            if (info[i].pixelCount != 0)
                empty = false;
        if (empty)
            continue;

        for (unsigned heap=0; heap<=1u; ++heap)
        {
            printf("\n|%-7s ||", (heap==0) ? implNames[impl] : "");
            for (unsigned omp=0; omp<=1u; ++omp)
            {
                for (unsigned map=0; map<=1u; ++map)
                {
                    const unsigned index = omp*4 + heap*2 + map;
                    {
                        if (info[index].pixelCount == 0)
                            printf("                     |");
                        else
                            printf(" %-6s %5.1f Mpix/s |", (heap)?"Heap:":"Stack:", double(info[index].pixelCount) / double(ticks_to_us(info[index].ticks)));
                    }
                }
            }
        }
       printf("\n|--------||---------------------|---------------------|---------------------|---------------------|");
    }
    printf("\n\n");

    return result;
}


void test_compute_ssim(const char* imgPath, const char* refPath, rmgr::ssim::Implementation impl, unsigned flags, bool buildSSimMap, RefSsim expectedSSIMs[])
{
    const unsigned supportedImpls = rmgr::ssim::select_impl(impl);
    ASSERT_NE(0u, supportedImpls & (1 << impl)); // Fails if instruction set not supported or support not enabled at build time
    
    typedef rmgr::ssim::uint8_t uint8_t;

    // Load images
    int imgWidth, imgHeight, imgChannels;
    uint8_t* imgData = stbi_load(imgPath, &imgWidth, &imgHeight, &imgChannels, 0);
    ASSERT_NE(static_cast<uint8_t*>(NULL), imgData);

    int refWidth, refHeight, refChannels;
    uint8_t* refData = stbi_load(refPath, &refWidth, &refHeight, &refChannels, 0);
    ASSERT_NE(static_cast<uint8_t*>(NULL), refData);

    ASSERT_EQ(refWidth,    imgWidth);
    ASSERT_EQ(refHeight,   imgHeight);
    ASSERT_EQ(refChannels, imgChannels);

    // Compute SSIM for each channel
    float* ssimMap = NULL;
    if (buildSSimMap)
    {
        ssimMap = new float[imgWidth * imgHeight];
        ASSERT_NE(ssimMap, static_cast<float*>(NULL));
    }

    PerfInfo& perfInfo = g_perfInfo[impl][flags * 2 + !!buildSSimMap];

    for (int channel=0; channel<refChannels; ++channel)
    {
        RefSsim& expected = expectedSSIMs[channel];
        if (expected.map == NULL)
        {
            // Compute reference SSIM using the naive implementation
            expected.map = new RefFloat[imgWidth * imgHeight];
            ASSERT_NE(static_cast<RefFloat*>(NULL), expected.map);
            expected.ssim = rmgr::ssim::naive::compute_ssim<RefFloat>(refWidth, refHeight, refData+channel, refChannels, refWidth*refChannels, imgData+channel, imgChannels, imgWidth*imgChannels, expected.map, 1, imgWidth);
        }

        Ticks t1 = get_ticks();
        const float ssim = rmgr::ssim::compute_ssim(refWidth, refHeight, refData+channel, refChannels, refWidth*refChannels, imgData+channel, imgChannels, imgWidth*imgChannels, ssimMap, 1, imgWidth, flags);
        Ticks t2 = get_ticks();
        perfInfo.ticks      += t2 - t1;
        perfInfo.pixelCount += imgWidth * imgHeight;

        g_largestGlobalError[impl] = std::max(g_largestGlobalError[impl], std::abs(expected.ssim - static_cast<RefFloat>(ssim)));
        EXPECT_NEAR(double(expected.ssim), double(ssim), GLOBAL_TOLERANCE);

        if (buildSSimMap)
        {
            for (int y=0; y<imgHeight; ++y)
            {
                for (int x=0; x<imgWidth; ++x)
                {
                    const RefFloat refPixSsim = expected.map[y*imgWidth + x];
                    const RefFloat pixSsim    = ssimMap[y*imgWidth + x];
                    g_largestPixelError[impl] = std::max(g_largestPixelError[impl], std::abs(refPixSsim - pixSsim));
                    ASSERT_NEAR(double(refPixSsim), double(pixSsim), PIXEL_TOLERANCE);
                }
            }
        }
    }

    delete[] ssimMap;

    // Free images
    stbi_image_free(refData);
    stbi_image_free(imgData);
}


static void test_einstein(rmgr::ssim::Implementation impl, unsigned flags, bool buildSsimMap)
{
    // These are the examples from the original SSIM page
//    static const float ssims[] = {1.0f, 0.988f, 0.913f, 0.840f, 0.694f, 0.662f};
    static char const* const files[] =
    {
        "einstein.png",
        "meanshift.png",
        "contrast.png",
        "impulse.png",
        "blur.png",
        "jpg.png"
    };

    const size_t fileCount = sizeof(files) / sizeof(files[0]);
    static RefSsim ssims[fileCount] = {}; //{1.000000f, 0.987346f, 0.901217f, 0.839534f, 0.702192f, 0.669938f};

    char refPath[256];
    snprintf(refPath, sizeof(refPath), "%s/%s", g_imagesDir, files[0]);
    for (size_t i=0; i<fileCount; ++i)
    {
        char imgPath[256];
        snprintf(imgPath, sizeof(imgPath), "%s/%s", g_imagesDir, files[i]);
        test_compute_ssim(imgPath, refPath, impl, flags, buildSsimMap, ssims+i);
    }
}


static void test_bbb(const char* stub, rmgr::ssim::Implementation impl, unsigned flags, bool buildSsimMap, RefSsim expectedSSIMs[11][3])
{
    char pngPath[256];
    snprintf(pngPath, sizeof(pngPath), "%s/%s.png", g_imagesDir, stub);

    for (int jpgQuality=0; jpgQuality<=100; jpgQuality+=10)
    {
        char jpgPath[256];
        snprintf(jpgPath, sizeof(jpgPath), "%s/%s_%02d.jpg", g_imagesDir, stub, jpgQuality);
        SCOPED_TRACE(jpgPath);
        test_compute_ssim(jpgPath, pngPath, impl, flags, buildSsimMap, *expectedSSIMs++);
    }
}


static void test_bbb360(rmgr::ssim::Implementation impl, unsigned flags, bool buildSsimMap)
{
    static RefSsim ssims[11][3] = {};
    test_bbb("big_buck_bunny_360_07806", impl, flags, buildSsimMap, ssims);
}


static void test_bbb1080(rmgr::ssim::Implementation impl, unsigned flags, bool buildSsimMap)
{
    static RefSsim ssims[11][3] = {};
    test_bbb("big_buck_bunny_1080_07806", impl, flags, buildSsimMap, ssims);
}


#define DO_TEST_IMPL(name, impl, IMPL, suffix, flags)                                                                      \
    TEST(name, impl##_stack##suffix)        {test_##name(rmgr::ssim::IMPL, (flags),                               true);}  \
    TEST(name, impl##_heap##suffix)         {test_##name(rmgr::ssim::IMPL, (flags)|rmgr::ssim::FLAG_HEAP_BUFFERS, true);}  \
    TEST(name, impl##_stack_nomap##suffix)  {test_##name(rmgr::ssim::IMPL, (flags),                               false);} \
    TEST(name, impl##_heap_nomap##suffix)   {test_##name(rmgr::ssim::IMPL, (flags)|rmgr::ssim::FLAG_HEAP_BUFFERS, false);}


#if RMGR_SSIM_USE_OPENMP
    #define TEST_IMPL(name, impl, IMPL)            \
        DO_TEST_IMPL(name, impl, IMPL, ,        0) \
        DO_TEST_IMPL(name, impl, IMPL, _openmp, rmgr::ssim::FLAG_OPENMP)
#else
    #define TEST_IMPL(name, impl, IMPL)  DO_TEST_IMPL(name, impl, IMPL, , 0)
#endif

#define TEST_AUTO(name)     TEST_IMPL(name, auto,    IMPL_AUTO)
#define TEST_GENERIC(name)  TEST_IMPL(name, generic, IMPL_GENERIC)

#if RMGR_ARCH_IS_X86_ANY
    #define TEST_X86(name)             \
        TEST_IMPL(name, sse, IMPL_SSE) \
        TEST_IMPL(name, avx, IMPL_AVX) \
        TEST_IMPL(name, fma, IMPL_FMA)
#else
    #define TEST_X86(name)
#endif

#if RMGR_ARCH_IS_ARM_ANY
    #define TEST_ARM(name)  TEST_IMPL(name, neon, IMPL_NEON)
#else
    #define TEST_ARM(name)
#endif

#define TEST_ALL(name) \
    TEST_GENERIC(name) \
    TEST_X86(name)     \
    TEST_ARM(name)     \
    TEST_AUTO(name)

TEST_ALL(einstein)
TEST_ALL(bbb360)
TEST_ALL(bbb1080)
