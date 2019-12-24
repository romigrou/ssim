#include <rmgr/ssim.h>
#include <gtest/gtest.h>


#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
RMGR_WARNING_PUSH()
RMGR_WARNING_MSVC_DISABLE(4100) // unreferenced formal parameter
#include "stb_image.h"
RMGR_WARNING_POP()


int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


void test_compute_ssim(const char* imgPath, const char* refPath, const float expectedSSIMs[], float tolerance=1e-4f)
{
    int imgWidth, imgHeight, imgChannels;
    uint8_t* imgData = stbi_load(imgPath, &imgWidth, &imgHeight, &imgChannels, 0);
    ASSERT_NE(nullptr, imgData);

    int refWidth, refHeight, refChannels;
    uint8_t* refData = stbi_load(refPath, &refWidth, &refHeight, &refChannels, 0);
    ASSERT_NE(nullptr, refData);

    ASSERT_EQ(refWidth,    imgWidth);
    ASSERT_EQ(refHeight,   imgHeight);
    ASSERT_EQ(refChannels, imgChannels);

    for (int channel=0; channel<refChannels; ++channel)
    {
        float ssim = rmgr::ssim::compute_ssim(refWidth, refHeight, refData+channel, refChannels, refWidth*refChannels, imgData+channel, imgChannels, imgWidth*imgChannels);
        EXPECT_NEAR(expectedSSIMs[channel], ssim, tolerance);
    }

    stbi_image_free(refData);
    stbi_image_free(imgData);
}


TEST(SSIM, einstein)
{
    // These are the examples from the original SSIM page
//    const float ssims[] = {1.0f, 0.988f, 0.913f, 0.840f, 0.694f, 0.662f};
    const float ssims[] = {1.000000f, 0.987346f, 0.901217f, 0.839534f, 0.702192f, 0.669938f};

    test_compute_ssim(RMGR_SSIM_TESTS_DIR "/images/einstein.png",  RMGR_SSIM_TESTS_DIR "/images/einstein.png", ssims+0);
    test_compute_ssim(RMGR_SSIM_TESTS_DIR "/images/meanshift.png", RMGR_SSIM_TESTS_DIR "/images/einstein.png", ssims+1);
    test_compute_ssim(RMGR_SSIM_TESTS_DIR "/images/contrast.png",  RMGR_SSIM_TESTS_DIR "/images/einstein.png", ssims+2);
    test_compute_ssim(RMGR_SSIM_TESTS_DIR "/images/impulse.png",   RMGR_SSIM_TESTS_DIR "/images/einstein.png", ssims+3);
    test_compute_ssim(RMGR_SSIM_TESTS_DIR "/images/blur.png",      RMGR_SSIM_TESTS_DIR "/images/einstein.png", ssims+4);
    test_compute_ssim(RMGR_SSIM_TESTS_DIR "/images/jpg.png",       RMGR_SSIM_TESTS_DIR "/images/einstein.png", ssims+5);
}


static void test_bbb(const char* stub, const float expectedSSIMs[][3])
{
    char pngPath[256];
    snprintf(pngPath, sizeof(pngPath), "%s/images/%s.png", RMGR_SSIM_TESTS_DIR, stub);

    for (int jpgQuality=0; jpgQuality<=100; jpgQuality+=10)
    {
        char jpgPath[256];
        snprintf(jpgPath, sizeof(jpgPath), "%s/images/%s_%02d.jpg", RMGR_SSIM_TESTS_DIR, stub, jpgQuality);
        SCOPED_TRACE(jpgPath);
        test_compute_ssim(jpgPath, pngPath, *expectedSSIMs++);
    }
}


TEST(SSIM, bbb360)
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

    test_bbb("big_buck_bunny_360_07806", ssims);
}


TEST(SSIM, bbb1080)
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

    test_bbb("big_buck_bunny_1080_07806", ssims);
}
