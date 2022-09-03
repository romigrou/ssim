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

#include "ssim_naive.h"
#include <rmgr/ssim.h>
#include <rmgr/ssim-openmp.h>
#include "../src/ssim_internal.h"
#include <gtest/gtest.h>


#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
RMGR_WARNING_PUSH()
RMGR_WARNING_GCC_DISABLE("-Wsign-compare")
#include "../src/stb_image.h"
RMGR_WARNING_POP()

#if RMGR_COMPILER_IS_MSVC
    #define snprintf  _snprintf_s
#endif


const unsigned IMPL_COUNT = 8;


static const char  g_defaultImagesDir[] = {RMGR_SSIM_TESTS_IMAGES_DIR};
static const char* g_imagesDir = g_defaultImagesDir;


// Floating-point type for computing reference SSIMs. The more precise, the better.
// This being said, 128-bit float being software is really too slow to be usable.
#if defined(__SIZEOF_FLOAT128__) && 0
    typedef __float128 RefFloat;
    #define REF_TOLERANCE  DBL_EPSILON
#elif DBL_MANT_DIG < LDBL_MANT_DIG && LDBL_MANT_DIG <= 64 && !RMGR_ARCH_IS_X86_64 // long double OK if no more than 80 bits (otherwise likely to be software)
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

using rmgr::ssim::uint64_t;


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

    static char const* const implNames[IMPL_COUNT] = {"Auto", "Generic", "SSE", "SSE2", "AVX", "FMA", "AVX-512", "Neon"};

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
                const long double pixelErrorAvg  = static_cast<long double>(g_pixelErrorSum[impl] / RefFloat(g_pixelCount[impl]));
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


void test_compute_ssim(const char* imgPath, const char* refPath, rmgr::ssim::Implementation impl, bool openmp, bool useHeap,
                       bool buildSSimMap, RefSsim expectedSSIMs[], int maxWidth=INT_MAX, int maxHeight=INT_MAX)
{
    const unsigned supportedImpls = rmgr::ssim::select_impl(impl);
    ASSERT_TRUE(supportedImpls & (1 << impl)) << "--- Unsupported instruction set ---"; // Fails if instruction set not supported or support not enabled at build time

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

    const int imgStride = imgWidth * imgChannels;
    const int refStride = refWidth * refChannels;
    imgWidth  = std::min(imgWidth,  maxWidth);
    imgHeight = std::min(imgHeight, maxHeight);

    // Compute SSIM for each channel
    float* ssimMap = NULL;
    if (buildSSimMap)
    {
        ssimMap = new float[imgWidth * imgHeight];
        ASSERT_NE(ssimMap, static_cast<float*>(NULL));
    }

    PerfInfo& perfInfo = g_perfInfo[impl][(4 * !!openmp) + (2 * !!useHeap) + !!buildSSimMap];

    rmgr::ssim::Params params = {};
    params.width      = imgWidth;
    params.height     = imgHeight;
    params.ssimMap    = ssimMap;
    params.ssimStep   = 1;
    params.ssimStride = imgWidth;
    if (useHeap)
        params.use_default_allocator();

    for (int channel=0; channel<refChannels; ++channel)
    {
        RefSsim& expected = expectedSSIMs[channel];
        if (expected.map == NULL)
        {
            // Compute reference per-pixel SSIM using the naive implementation
            expected.map = new RefFloat[imgWidth * imgHeight];
            ASSERT_NE(static_cast<RefFloat*>(NULL), expected.map);
            const RefFloat refSsim = rmgr::ssim::naive::compute_ssim<RefFloat>(imgWidth, imgHeight, refData+channel, refChannels, refStride, imgData+channel, imgChannels, imgStride, expected.map, 1, imgWidth);
            if (expected.ssim == 0)
                expected.ssim = refSsim;
            else
            {
                ASSERT_NEAR(double(expected.ssim), double(refSsim), REF_TOLERANCE);
            }
        }

        params.imgA.init_interleaved(refData, refStride, refChannels, channel);
        params.imgB.init_interleaved(imgData, imgStride, imgChannels, channel);

        Ticks t1 = get_ticks();
        float ssim;
        if (openmp)
            ssim = rmgr::ssim::compute_ssim_openmp(params);
        else
            ssim = rmgr::ssim::compute_ssim(params);
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


static void test_einstein(rmgr::ssim::Implementation impl, bool openmp, bool useHeap, bool buildSsimMap)
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
        test_compute_ssim(imgPath, refPath, impl, openmp, useHeap, buildSsimMap, ssims+i);
    }
}


static void test_bbb(const char* stub, rmgr::ssim::Implementation impl, bool openmp, bool useHeap, bool buildSsimMap, RefSsim expectedSSIMs[11][3], int maxWidth=INT_MAX, int maxHeight=INT_MAX)
{
    char pngPath[256];
    snprintf(pngPath, sizeof(pngPath), "%s/%s.png", g_imagesDir, stub);

    for (int jpgQuality=0; jpgQuality<=100; jpgQuality+=10)
    {
        char jpgPath[256];
        snprintf(jpgPath, sizeof(jpgPath), "%s/%s_%02d.jpg", g_imagesDir, stub, jpgQuality);
        SCOPED_TRACE(jpgPath);
        test_compute_ssim(jpgPath, pngPath, impl, openmp, useHeap, buildSsimMap, *expectedSSIMs++, maxWidth, maxHeight);
    }
}


static void test_bbb360(rmgr::ssim::Implementation impl, bool openmp, bool useHeap, bool buildSsimMap)
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
    test_bbb("big_buck_bunny_360_07806", impl, openmp, useHeap, buildSsimMap, ssims);
}


static void test_bbb1080(rmgr::ssim::Implementation impl, bool openmp, bool useHeap, bool buildSsimMap)
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
    test_bbb("big_buck_bunny_1080_07806", impl, openmp, useHeap, buildSsimMap, ssims);
}


static void test_bbb255(rmgr::ssim::Implementation impl, bool openmp, bool useHeap, bool buildSsimMap)
{
    static RefSsim ssims[11][3] =
    {
        REF_SSIM3(0.547853090228261106999379276950104,0.529623594694831156206300716496893,0.472084323083059488498646172724762),
        REF_SSIM3(0.809755523675252822685244853927566,0.814426818840510850996111381464533,0.544933203848316557894374652249962),
        REF_SSIM3(0.876014757295356616704665201587882,0.897523856877390793723396460382586,0.632972315485404370746008931385895),
        REF_SSIM3(0.914664991462819263939303286362269,0.931120381966295425892702540880713,0.686109142654913901186202720785575),
        REF_SSIM3(0.928977009556447463131367429121640,0.945766807911778849341705040670454,0.727579272380384194954380769836644),
        REF_SSIM3(0.938670961960028521854841434395655,0.955933222964943924304459440580605,0.752330427496336116281642417390214),
        REF_SSIM3(0.947768675802921839807904323922269,0.963626546372889349103117767560239,0.777366975027434040530955007691513),
        REF_SSIM3(0.958264137101390816066691428828457,0.971975375482976738047657713104747,0.807678352183838779533372629229701),
        REF_SSIM3(0.969329249948856804039916374628339,0.979914844910674473710556058510994,0.846762346538395931401516471211991),
        REF_SSIM3(0.980592553796793365988164175595120,0.988459119584828780495244400755345,0.895953581733139167807368721794607),
        REF_SSIM3(0.995554150150431506932530273805506,0.998351279973895838869633438367112,0.990122638569351284864292289804271)
    };
    test_bbb("big_buck_bunny_360_07806", impl, openmp, useHeap, buildSsimMap, ssims, 255, 63);
}


static void test_bbb257(rmgr::ssim::Implementation impl, bool openmp, bool useHeap, bool buildSsimMap)
{
    static RefSsim ssims[11][3] =
    {
        REF_SSIM3(0.542276707562074769581575599299554, 0.525113322548177231515318063403652, 0.464599048886867081950443996899369),
        REF_SSIM3(0.805810181415264423899147244107073, 0.810438721532292428451728084038901, 0.538293622407871291931259304000566),
        REF_SSIM3(0.872735533853739628519858014286426, 0.895182776723071253739721667641047, 0.630193454725971967298039991894668),
        REF_SSIM3(0.913002101952119352928010490430082, 0.928624128659417541388364538875253, 0.684906731917911622139610390093970),
        REF_SSIM3(0.927169970713589035010921470404810, 0.944665368923434561816151321508847, 0.724496131887187528805858804992732),
        REF_SSIM3(0.937157885044158319137038497807958, 0.955175071708954809479906205323521, 0.748670938897749880847489895167691),
        REF_SSIM3(0.946633039018331466914059129869777, 0.963013990787300494540990775839417, 0.774412837971718501004941248762081),
        REF_SSIM3(0.957419450388110094765620208566235, 0.971555465640791070282699292589272, 0.804843672607972800364956814239845),
        REF_SSIM3(0.968682363715465700009427132545245, 0.979638337108377049907914623084336, 0.845341168878976310835346487676230),
        REF_SSIM3(0.980170770627140053380079376401973, 0.988310804353821863074583296682297, 0.895185108784727410716995229679136),
        REF_SSIM3(0.995509458704857621204542425874914, 0.998331030868806375721824867748273, 0.990080646740585025047375788822065)
    };
    test_bbb("big_buck_bunny_360_07806", impl, openmp, useHeap, buildSsimMap, ssims, 257, 65);
}


#define DO_TEST_IMPL(name, impl, IMPL, suffix, omp)                                             \
    TEST(name, impl##_stack##suffix)        {test_##name(rmgr::ssim::IMPL, omp, false, true);}  \
    TEST(name, impl##_heap##suffix)         {test_##name(rmgr::ssim::IMPL, omp, true,  true);}  \
    TEST(name, impl##_stack_nomap##suffix)  {test_##name(rmgr::ssim::IMPL, omp, false, false);} \
    TEST(name, impl##_heap_nomap##suffix)   {test_##name(rmgr::ssim::IMPL, omp, true,  false);}


#if RMGR_SSIM_USE_OPENMP
    #define TEST_IMPL(name, impl, IMPL)                \
        DO_TEST_IMPL(name, impl, IMPL, ,        false) \
        DO_TEST_IMPL(name, impl, IMPL, _openmp, true)
#else
    #define TEST_IMPL(name, impl, IMPL)  DO_TEST_IMPL(name, impl, IMPL, , false, 0)
#endif

#define TEST_AUTO(name)     TEST_IMPL(name, auto,    IMPL_AUTO)
#define TEST_GENERIC(name)  TEST_IMPL(name, generic, IMPL_GENERIC)

#if RMGR_ARCH_IS_X86_ANY
    #define TEST_X86(name)                 \
        TEST_IMPL(name, sse,    IMPL_SSE)  \
        TEST_IMPL(name, sse2,   IMPL_SSE2) \
        TEST_IMPL(name, avx,    IMPL_AVX)  \
        TEST_IMPL(name, fma,    IMPL_FMA)  \
        TEST_IMPL(name, avx512, IMPL_AVX512)
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
TEST_ALL(bbb255)
TEST_ALL(bbb257)
TEST_ALL(bbb360)
TEST_ALL(bbb1080)
