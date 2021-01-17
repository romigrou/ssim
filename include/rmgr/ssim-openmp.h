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

#ifndef RMGR_SSIM_OMP_H
#define RMGR_SSIM_OMP_H


#include <rmgr/ssim.h>


namespace rmgr { namespace ssim
{


/**
 * @brief Computes in multi-thread the SSIM of a single channel of two images and, optionally, the per-pixel SSIM map
 *
 * This function can handle almost any kind of image storage scheme (interleaved or planar, top-down or
 * bottom-up, row-major or column-major, ...). All you need is to specify the `step` and `stride` parameters
 * such that the address of a pixel's channel is given by `imgData + x * imgStep + y * imgStride` with `x` and
 * `y` being the horizontal and vertical coordinates, respectively.
 *
 * @note The SSIM does not depend on the order of traversal of the images, so you can safely swap the `step` and `stride`
 *       parameters if this improves cache hit rates, as long as both images are traversed in the same order.
 *
 * @note Multi-threading support implemented via OpenMP (hence the name)
 *
 * @param [in]  width      The images' width,  in pixels
 * @param [in]  height     The images' height, in pixels
 * @param [in]  imgAData   A pointer to the considered channel of the top-left pixel of image A.
 * @param [in]  imgAStep   For image A, the distance (in bytes) between a pixel and the one immediately to its right.
 *                         This distance may be negative.
 * @param [in]  imgAStride For image A, the distance (in bytes) between a pixel and the one immediately below it.
 *                         This distance may be negative.
 * @param [in]  imgBData   A pointer to the considered channel of the top-left pixel of image B.
 * @param [in]  imgBStep   For image B, the distance (in bytes) between a pixel and the one immediately to its right.
 *                         This distance may be negative.
 * @param [in]  imgBStride For image B, the distance (in bytes) between a pixel and the one immediately below it.
 *                         This distance may be negative.
 * @param [out] ssimMap    A pointer to the top-left pixel the SSIM map. You can set this to `NULL` if you don't need
 *                         the SSIM map, in which case the `ssimStep` and `ssimStride` parameters will be ignored.
 * @param [in]  ssimStep   The distance (in `float`s) between a pixel's SSIM and that of the pixel immediately to its right.
 *                         This distance may be negative.
 * @param [in]  ssimStride The distance (in `float`s) between a pixel's SSIM and that of the pixel immediately below it.
 *                         This distance may be negative.
 * @param [in]  flags      A set of optional flags to tweak the behaviour of the function
 *
 * @retval >=0 The image's SSIM, in the range [0;1].
 * @retval <0  An error occurred, call `get_errno()` to retrieve the error number.
 */
float compute_ssim_openmp(uint32_t width, uint32_t height,
                          const uint8_t* imgAData, ptrdiff_t imgAStep, ptrdiff_t imgAStride,
                          const uint8_t* imgBData, ptrdiff_t imgBStep, ptrdiff_t imgBStride,
                          float* ssimMap, ptrdiff_t ssimStep, ptrdiff_t ssimStride, unsigned flags=0) RMGR_NOEXCEPT;


/**
 * @brief Computes in multi-thread the SSIM of a single channel of two images
 *
 * This function can handle almost any kind of image storage scheme (interleaved or planar, top-down or
 * bottom-up, row-major or column-major, ...). All you need is to specify the `step` and `stride` parameters
 * such that the address of a pixel's channel is given by `imgData + x * imgStep + y * imgStride` with `x` and
 * `y` being the horizontal and vertical coordinates, respectively.
 *
 * @note The SSIM does not depend on the order of traversal of the images, so you can safely swap the `step` and `stride`
 *       parameters if this improves cache hit rates, as long as both images are traversed in the same order.
 *
 * @param [in] width      The images' width,  in pixels
 * @param [in] height     The images' height, in pixels
 * @param [in] imgAData   A pointer to the considered channel of the top-left pixel of image A.
 * @param [in] imgAStep   For image A, the distance (in bytes) between a pixel and the one immediately to its right.
 *                        This distance may be negative.
 * @param [in] imgAStride For image A, the distance (in bytes) between a pixel and the one immediately below it.
 *                        This distance may be negative.
 * @param [in] imgBData   A pointer to the considered channel of the top-left pixel of image B.
 * @param [in] imgBStep   For image B, the distance (in bytes) between a pixel and the one immediately to its right.
 *                        This distance may be negative.
 * @param [in] imgBStride For image B, the distance (in bytes) between a pixel and the one immediately below it.
 *                        This distance may be negative.
 * @param [in] flags      A set of optional flags to tweak the behaviour of the function
 *
 * @retval >=0 The image's SSIM, in the range [0;1].
 * @retval <0  An error occurred, call `get_errno()` to retrieve the error number.
 */
inline float compute_ssim_openmp(uint32_t width, uint32_t height,
                              const uint8_t* imgAData, ptrdiff_t imgAStep, ptrdiff_t imgAStride,
                              const uint8_t* imgBData, ptrdiff_t imgBStep, ptrdiff_t imgBStride, unsigned flags=0) RMGR_NOEXCEPT
{
    return compute_ssim_openmp(width, height, imgAData, imgAStep, imgAStride, imgBData, imgBStep, imgBStride, NULL, 0, 0, flags);
}


}} // namespace rmgr::ssim

#endif // RMGR_SSIM_H
