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
#if RMGR_SSIM_USE_OPENMP
    #include <rmgr/ssim-openmp.h>
#endif

#include <cstddef>
#include <cstdio>
#include <cstring>


#define STB_IMAGE_IMPLEMENTATION
#define STBI_FAILURE_USERMSG
RMGR_WARNING_PUSH()
RMGR_WARNING_MSVC_DISABLE(4244) // conversion from 'type1' to 'type2', possible loss of data
RMGR_WARNING_GCC_DISABLE("-Wsign-compare")
RMGR_WARNING_GCC_DISABLE("-Wunused-but-set-variable")
#include "stb_image.h"
RMGR_WARNING_POP()


extern "C" int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        fprintf(stderr, "Usage: rmgr-ssim-sample file1 file2\n\n");
        return EXIT_FAILURE;
    }

    const char* img1Path = argv[1];
    const char* img2Path = argv[2];

    // Load 1st image
    int img1Width, img1Height, img1ChannelCount;
    stbi_uc* img1 = stbi_load(img1Path, &img1Width, &img1Height, &img1ChannelCount, 0);
    if (img1 == NULL)
    {
        fprintf(stderr, "Failed to load image \"%s\": %s\n", img1Path, stbi_failure_reason());
        return EXIT_FAILURE;
    }

    // Load 2nd image
    int img2Width, img2Height, img2ChannelCount;
    stbi_uc* img2 = stbi_load(img2Path, &img2Width, &img2Height, &img2ChannelCount, 0);
    if (img2 == NULL)
    {
        fprintf(stderr, "Failed to load image \"%s\": %s\n", img2Path, stbi_failure_reason());
        return EXIT_FAILURE;
    }

    // Check images have the same features
    if (img1Width != img2Width || img1Height != img2Height || img1ChannelCount != img2ChannelCount)
    {
        fprintf(stderr, "Images must have the same dimensions and number of channels\n");
        return EXIT_FAILURE;
    }

    // Compute SSIM of each channel
    rmgr::ssim::Params params = {};
    params.width  = img1Width;
    params.height = img1Height;
    for (int channelNum=0; channelNum < img1ChannelCount; ++channelNum)
    {
        params.imgA.init_interleaved(img1, img1Width*img1ChannelCount, channelNum, img1ChannelCount);
        params.imgB.init_interleaved(img2, img2Width*img2ChannelCount, channelNum, img2ChannelCount);
#if RMGR_SSIM_USE_OPENMP
        const float ssim = rmgr::ssim::compute_ssim_openmp(params);
#else
        const float ssim = rmgr::ssim::compute_ssim(params);
#endif
        if (rmgr::ssim::get_errno(ssim) != 0)
            fprintf(stderr, "Failed to compute SSIM of channel %d\n", channelNum+1);
        else
            printf("SSIM of channel %d:% 7.4f\n", channelNum+1, ssim);
    }

    // Clean up
    stbi_image_free(img2);
    stbi_image_free(img1);

    return EXIT_SUCCESS;
}
