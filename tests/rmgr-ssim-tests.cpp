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


// Floating-point type for computing reference SSIMs. The more precise, the better.
// This being said, 128-bit float being software is really too slow to be usable.
#if defined(__SIZEOF_FLOAT128__) && 0
    typedef __float128 RefFloat;
    #define REF_TOLERANCE  DBL_EPSILON
#elif DBL_MANT_DIG < LDBL_MANT_DIG && LDBL_MANT_DIG <= 64 // long double OK if no more than 80 bits (otherwise likely to be software)
    typedef long double RefFloat;
    #define REF_TOLERANCE  DBL_EPSILON
#else
    typedef double RefFloat;
    #define REF_TOLERANCE  1e-13
#endif


struct RefSsim
{
    RefFloat  ssim;
    RefFloat* map;

    ~RefSsim() {delete[] map;}
};


#define REF_SSIM(val)                {RefFloat(val##L), NULL}
#define REF_SSIM3(val1, val2, val3)  {REF_SSIM(val1), REF_SSIM(val2), REF_SSIM(val3)}


static RefFloat g_globalErrorMax[IMPL_COUNT] = {};
static RefFloat g_globalErrorSum[IMPL_COUNT] = {};
static RefFloat g_pixelErrorMax[IMPL_COUNT]  = {};
static RefFloat g_pixelErrorSum[IMPL_COUNT]  = {};
static uint64_t g_imageCount[IMPL_COUNT]     = {};
static uint64_t g_pixelCount[IMPL_COUNT]     = {};

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
           "         ||======================|======================|======================|======================|\n"
           "         ||   Avg. Global Error  |   Max. Global Error  | Avg. Per-Pixel Error | Max. Per-Pixel Error |\n"
           "|========||======================|======================|======================|======================|\n");
    for (unsigned impl=0; impl<IMPL_COUNT; ++impl)
    {
        if (g_imageCount[impl] != 0)
        {
            const long double globalErrorAvg = static_cast<long double>(g_globalErrorSum[impl] / RefFloat(g_imageCount[impl]));
            const long double globalErrorMax = static_cast<long double>(g_globalErrorMax[impl]);
            printf("|%-7s || %16.14Le | %16.14Le |", implNames[impl], globalErrorAvg, globalErrorMax);
            if (g_pixelCount[impl] != 0)
            {
                const long double pixelErrorAvg  = static_cast<long double>(g_pixelErrorSum[impl]  / RefFloat(g_pixelCount[impl]));
                const long double pixelErrorMax  = static_cast<long double>(g_pixelErrorMax[impl]);
                printf(" %16.14Le | %16.14Le |\n", pixelErrorAvg, pixelErrorMax);
            }
            else
                printf("                      |                      |\n");
        }
    }
    printf("|========||======================|======================|======================|======================|\n\n");

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
            const RefFloat refSsim = rmgr::ssim::naive::compute_ssim<RefFloat>(refWidth, refHeight, refData+channel, refChannels, refWidth*refChannels, imgData+channel, imgChannels, imgWidth*imgChannels, expected.map, 1, imgWidth);
            if (expected.ssim == 0)
                expected.ssim = refSsim;
            else
            {
                ASSERT_NEAR(double(expected.ssim), double(refSsim), REF_TOLERANCE);
            }
        }

        Ticks t1 = get_ticks();
        const float ssim = rmgr::ssim::compute_ssim(refWidth, refHeight, refData+channel, refChannels, refWidth*refChannels, imgData+channel, imgChannels, imgWidth*imgChannels, ssimMap, 1, imgWidth, flags);
        Ticks t2 = get_ticks();
        perfInfo.ticks      += t2 - t1;
        perfInfo.pixelCount += imgWidth * imgHeight;

        const RefFloat globalError = std::abs(expected.ssim - static_cast<RefFloat>(ssim));
        g_globalErrorSum[impl] += globalError;
        g_globalErrorMax[impl]  = std::max(g_globalErrorMax[impl], globalError);
        g_imageCount[impl]++;

        EXPECT_NEAR(double(expected.ssim), double(ssim), GLOBAL_TOLERANCE);

        if (buildSSimMap)
        {
            g_pixelCount[impl] += imgWidth * imgHeight;
            for (int y=0; y<imgHeight; ++y)
            {
                for (int x=0; x<imgWidth; ++x)
                {
                    const RefFloat refPixSsim = expected.map[y*imgWidth + x];
                    const RefFloat pixSsim    = ssimMap[y*imgWidth + x];
                    const RefFloat pixError   = std::abs(refPixSsim - pixSsim);
                    g_pixelErrorSum[impl] += pixError;
                    g_pixelErrorMax[impl]  = std::max(g_pixelErrorMax[impl], pixError);
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
    static RefSsim ssims[fileCount] =
    {
        REF_SSIM(1.000000000000000000000000000000000),
        REF_SSIM(0.987345868581455342542598819456431),
        REF_SSIM(0.901217091012390185892926336265424),
        REF_SSIM(0.839533769204009687363862456348761),
        REF_SSIM(0.702192033056262932311859850040160),
        REF_SSIM(0.669938383706498006524758818118705)
    };

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
    static RefSsim ssims[11][3] =
    {
        REF_SSIM3(0.536721290892722071348429184055824, 0.557679233053209125289696932926184, 0.526031513635702840587504290945777),
        REF_SSIM3(0.781960192572926695821628595832862, 0.804149451534862476018243063427270, 0.703025357253354342054715574458660),
        REF_SSIM3(0.862834074806723617859990108052267, 0.880701746915744172950579035555195, 0.784356475703502210131566701543072),
        REF_SSIM3(0.898070496358702480378970886438265, 0.912741992286503936498222222880013, 0.822446210626770627074389393816049),
        REF_SSIM3(0.915514377758508385333412880473089, 0.928698340210038737751945920509119, 0.847240538058324639504848334862311),
        REF_SSIM3(0.927101962500570103255129350527244, 0.938759964612735267835543684222428, 0.863675298922732847384112195637017),
        REF_SSIM3(0.936861175428972248097542675595024, 0.947408994087814754831062903822682, 0.877975192980452644830044205293250),
        REF_SSIM3(0.948039421285792285290951129294839, 0.957243672039308879048653965986035, 0.894392895809706693966294449559374),
        REF_SSIM3(0.960025033443250417857130228422079, 0.967731421658366681581874713168021, 0.913111040515580765780971175775347),
        REF_SSIM3(0.975073770491180460911502834692628, 0.981018317368407488675155659617660, 0.941232778962792419573339542702927),
        REF_SSIM3(0.996208334080668590937537440614104, 0.997984057353425511310232130540623, 0.993268256918489063772002792895026)
    };
    test_bbb("big_buck_bunny_360_07806", impl, flags, buildSsimMap, ssims);
}


static void test_bbb1080(rmgr::ssim::Implementation impl, unsigned flags, bool buildSsimMap)
{
    static RefSsim ssims[11][3] =
    {
        REF_SSIM3(0.632704310179284323736224768455510, 0.656142764394962996739214519731364, 0.623695432557639683843678064627403),
        REF_SSIM3(0.812020489871488534556205111362622, 0.833806350787162357242352648919526, 0.759281794789269749030825788509856),
        REF_SSIM3(0.887343184585925132481776122903426, 0.900193557177298719130396331548299, 0.839841203125112854756330457179141),
        REF_SSIM3(0.917532862518057468854291088845580, 0.927150885011434565745110388723764, 0.875725897714636173920241955383917),
        REF_SSIM3(0.932316370087100937494068233667649, 0.940758268869992301433085498771311, 0.895929166769288585507716547837280),
        REF_SSIM3(0.941901694591651686344008052769326, 0.949613464749596873985681059085839, 0.910315547642215924113321336475848),
        REF_SSIM3(0.950265536020765764264251404832098, 0.956867349069571310531406858105857, 0.921948002082147397407795790764632),
        REF_SSIM3(0.958682608575531426674052200491735, 0.964999380763535714345473472424178, 0.935168454162926873279122957566530),
        REF_SSIM3(0.968258734731813074663517781413743, 0.973531508679950274656887258961033, 0.949308067470548414147073057254927),
        REF_SSIM3(0.979172615495337753448454645197664, 0.983939620571677113813891515252583, 0.966121145307528297945637519451156),
        REF_SSIM3(0.994401970543854668870545571477672, 0.996821890450395867787859114090260, 0.991326411687471950314318376177629)
    };
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
