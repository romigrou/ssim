#include <rmgr/ssim.h>
#include "../src/ssim_internal.h"
#include <gtest/gtest.h>


#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
RMGR_WARNING_PUSH()
RMGR_WARNING_MSVC_DISABLE(4100) // unreferenced formal parameter
#include "stb_image.h"
RMGR_WARNING_POP()


extern "C" int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


namespace rmgr { namespace ssim
{
    void gaussian_blur(Float* dest, ptrdiff_t destStride, const Float* srce, ptrdiff_t srceStride, int32_t width, int32_t height, const Float kernel[], int radius) RMGR_NOEXCEPT;

    double sum_tile(uint32_t tileWidth, uint32_t tileHeight, uint32_t tileStride, Float c1, Float c2,
                    const Float* muATile, const Float* muBTile, const Float* sigmaA2Tile, const Float* sigmaB2Tile, const Float* sigmaABTile,
                    float* ssimTile, ptrdiff_t ssimStep, ptrdiff_t ssimStride) RMGR_NOEXCEPT;

    extern volatile GaussianBlurFct g_gaussianBlurFct;
    extern volatile SumTileFct      g_sumTileFct;
}}


enum Implementation
{
    IMPL_AUTO,
    IMPL_GENERIC,
    IMPL_SSE,
    IMPL_AVX
};


#ifdef _WIN32
    #include <windows.h>
    #ifdef _MSC_VER
        typedef unsigned __int64 uint64_t;
    #endif

    static inline uint64_t get_perf_counter() RMGR_NOEXCEPT
    {
        LARGE_INTEGER counter;
        QueryPerformanceCounter(&counter);
        return counter.QuadPart;
    }

    static uint64_t perf_counter_to_us(uint64_t counter) RMGR_NOEXCEPT
    {
        LARGE_INTEGER freq;
        QueryPerformanceFrequency(&freq);
        return (counter * 1000000) / freq.QuadPart;
    }
#else
    #include <time.h>

    static inline uint64_t get_perf_counter() RMGR_NOEXCEPT
    {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return uint64_t(ts.tv_sec) * 1000000000u + ts.tv_nsec;
    }

    static uint64_t perf_counter_to_us(uint64_t counter) RMGR_NOEXCEPT
    {
        return counter / 1000u;
    }
#endif


void test_compute_ssim(uint64_t& elapsed, uint64_t& pixelCount, const char* imgPath, const char* refPath, Implementation impl, unsigned flags, const float expectedSSIMs[], float tolerance=1e-4f)
{
    // Set implementation
    if (impl == IMPL_AUTO)
    {
        rmgr::ssim::g_gaussianBlurFct = NULL;
        rmgr::ssim::g_sumTileFct      = NULL;
    }
    else if (impl == IMPL_GENERIC)
    {
        rmgr::ssim::g_gaussianBlurFct = rmgr::ssim::gaussian_blur;
        rmgr::ssim::g_sumTileFct      = rmgr::ssim::sum_tile;
    }
    else if (impl == IMPL_SSE)
    {
#if RMGR_ARCH_IS_X86_ANY
        ASSERT_NE(static_cast<rmgr::ssim::GaussianBlurFct>(NULL), rmgr::ssim::sse::g_gaussianBlurFct);
        rmgr::ssim::g_gaussianBlurFct = rmgr::ssim::sse::g_gaussianBlurFct;
        rmgr::ssim::g_sumTileFct      = rmgr::ssim::sse::g_sumTileFct;
#else
        ASSERT_TRUE(false);
#endif
    }
    else if (impl == IMPL_AVX)
    {
#if RMGR_ARCH_IS_X86_ANY
        ASSERT_NE(static_cast<rmgr::ssim::GaussianBlurFct>(NULL), rmgr::ssim::avx::g_gaussianBlurFct);
        rmgr::ssim::g_gaussianBlurFct = rmgr::ssim::avx::g_gaussianBlurFct;
        rmgr::ssim::g_sumTileFct      = rmgr::ssim::sum_tile;
#else
        ASSERT_TRUE(false);
#endif
    }
    
    // Load images
    int imgWidth, imgHeight, imgChannels;
    uint8_t* imgData = stbi_load(imgPath, &imgWidth, &imgHeight, &imgChannels, 0);
    ASSERT_NE(nullptr, imgData);

    int refWidth, refHeight, refChannels;
    uint8_t* refData = stbi_load(refPath, &refWidth, &refHeight, &refChannels, 0);
    ASSERT_NE(nullptr, refData);

    ASSERT_EQ(refWidth,    imgWidth);
    ASSERT_EQ(refHeight,   imgHeight);
    ASSERT_EQ(refChannels, imgChannels);

    // Compute SSIM for each channel
    float* ssimMap = new float[imgWidth * imgHeight];
    for (int channel=0; channel<refChannels; ++channel)
    {
        uint64_t t1 = get_perf_counter();
        float ssim = rmgr::ssim::compute_ssim(refWidth, refHeight, refData+channel, refChannels, refWidth*refChannels, imgData+channel, imgChannels, imgWidth*imgChannels, ssimMap, 1, imgWidth, flags);
        uint64_t t2 = get_perf_counter();
        elapsed += t2 - t1;
        EXPECT_NEAR(expectedSSIMs[channel], ssim, tolerance);
    }

    pixelCount += imgWidth * imgHeight * imgChannels;
    delete[] ssimMap;

    // Free images
    stbi_image_free(refData);
    stbi_image_free(imgData);
}


static void test_einstein(Implementation impl, unsigned flags)
{
    // These are the examples from the original SSIM page
//    const float ssims[] = {1.0f, 0.988f, 0.913f, 0.840f, 0.694f, 0.662f};
    const float ssims[] = {1.000000f, 0.987346f, 0.901217f, 0.839534f, 0.702192f, 0.669938f};

    uint64_t elapsed    = 0;
    uint64_t pixelCount = 0;
    test_compute_ssim(elapsed, pixelCount, RMGR_SSIM_TESTS_DIR "/images/einstein.png",  RMGR_SSIM_TESTS_DIR "/images/einstein.png", impl, flags, ssims+0);
    test_compute_ssim(elapsed, pixelCount, RMGR_SSIM_TESTS_DIR "/images/meanshift.png", RMGR_SSIM_TESTS_DIR "/images/einstein.png", impl, flags, ssims+1);
    test_compute_ssim(elapsed, pixelCount, RMGR_SSIM_TESTS_DIR "/images/contrast.png",  RMGR_SSIM_TESTS_DIR "/images/einstein.png", impl, flags, ssims+2);
    test_compute_ssim(elapsed, pixelCount, RMGR_SSIM_TESTS_DIR "/images/impulse.png",   RMGR_SSIM_TESTS_DIR "/images/einstein.png", impl, flags, ssims+3);
    test_compute_ssim(elapsed, pixelCount, RMGR_SSIM_TESTS_DIR "/images/blur.png",      RMGR_SSIM_TESTS_DIR "/images/einstein.png", impl, flags, ssims+4);
    test_compute_ssim(elapsed, pixelCount, RMGR_SSIM_TESTS_DIR "/images/jpg.png",       RMGR_SSIM_TESTS_DIR "/images/einstein.png", impl, flags, ssims+5);
    double throughput = double(pixelCount) / double(perf_counter_to_us(elapsed));
    printf("  Throughput: %3.1f Mpix/s\n", throughput);
}

static void test_bbb(const char* stub, Implementation impl, unsigned flags, const float expectedSSIMs[][3])
{
    char pngPath[256];
    snprintf(pngPath, sizeof(pngPath), "%s/images/%s.png", RMGR_SSIM_TESTS_DIR, stub);

    uint64_t elapsed    = 0;
    uint64_t pixelCount = 0;
    for (int jpgQuality=0; jpgQuality<=100; jpgQuality+=10)
    {
        char jpgPath[256];
        snprintf(jpgPath, sizeof(jpgPath), "%s/images/%s_%02d.jpg", RMGR_SSIM_TESTS_DIR, stub, jpgQuality);
        SCOPED_TRACE(jpgPath);
        test_compute_ssim(elapsed, pixelCount, jpgPath, pngPath, impl, flags, *expectedSSIMs++);
    }
    double throughput = double(pixelCount) / double(perf_counter_to_us(elapsed));
    printf("  Throughput: %3.1f Mpix/s\n", throughput);
}

static void test_bbb360(Implementation impl, unsigned flags)
{
    static const float ssims[][3] =
    {
        //   R          G          B
        {0.536721f, 0.557679f, 0.526031f}, // q0
        {0.781960f, 0.804149f, 0.703025f}, // q10
        {0.862834f, 0.880702f, 0.784356f}, // q20
        {0.898071f, 0.912742f, 0.822446f}, // q30
        {0.915514f, 0.928698f, 0.847241f}, // q40
        {0.927102f, 0.938760f, 0.863675f}, // q50
        {0.936861f, 0.947409f, 0.877975f}, // q60
        {0.948039f, 0.957244f, 0.894393f}, // q70
        {0.960025f, 0.967731f, 0.913111f}, // q80
        {0.975074f, 0.981018f, 0.941233f}, // q90
        {0.996208f, 0.997984f, 0.993268f}  // q100
    };

    test_bbb("big_buck_bunny_360_07806", impl, flags, ssims);
}


static void test_bbb1080(Implementation impl, unsigned flags)
{
    static const float ssims[][3] =
    {
        //   R          G          B
        {0.632704f, 0.656143f, 0.623695f}, // q0
        {0.812020f, 0.833806f, 0.759282f}, // q10
        {0.887343f, 0.900194f, 0.839841f}, // q20
        {0.917533f, 0.927151f, 0.875726f}, // q30
        {0.932316f, 0.940758f, 0.895929f}, // q40
        {0.941902f, 0.949613f, 0.910316f}, // q50
        {0.950266f, 0.956867f, 0.921948f}, // q60
        {0.958683f, 0.964999f, 0.935168f}, // q70
        {0.968259f, 0.973531f, 0.949308f}, // q80
        {0.979173f, 0.983940f, 0.966121f}, // q90
        {0.994402f, 0.996822f, 0.991326f}  // q100
    };

    test_bbb("big_buck_bunny_1080_07806", impl, flags, ssims);
}


#if RMGR_SSIM_USE_OPENMP
    #define TEST_IMPL(name, impl, IMPL)                                                     \
        TEST(name, impl##_stack)        {test_##name(IMPL, 0);}                             \
        TEST(name, impl##_heap)         {test_##name(IMPL, rmgr::ssim::FLAG_HEAP_BUFFERS);} \
        TEST(name, impl##_stack_openmp) {test_##name(IMPL, rmgr::ssim::FLAG_OPENMP);}       \
        TEST(name, impl##_heap_openmp)  {test_##name(IMPL, rmgr::ssim::FLAG_HEAP_BUFFERS|rmgr::ssim::FLAG_OPENMP);}
#else
    #define TEST_IMPL(name, impl, IMPL)                                                     \
        TEST(name, impl##_stack)        {test_##name(IMPL, 0);}                             \
        TEST(name, impl##_heap)         {test_##name(IMPL, rmgr::ssim::FLAG_HEAP_BUFFERS);}
#endif

#define TEST_AUTO(name)     TEST_IMPL(name, auto,    IMPL_AUTO)
#define TEST_GENERIC(name)  TEST_IMPL(name, generic, IMPL_GENERIC)

#if RMGR_ARCH_IS_X86_ANY
    #define TEST_X86(name)             \
        TEST_IMPL(name, sse, IMPL_SSE) \
        TEST_IMPL(name, avx, IMPL_AVX)
#else
    #define TEST_X86(name)
#endif

#define TEST_ALL(name) \
    TEST_GENERIC(name) \
    TEST_X86(name)     \
    TEST_AUTO(name)

TEST_ALL(einstein)
TEST_ALL(bbb360)
TEST_ALL(bbb1080)
