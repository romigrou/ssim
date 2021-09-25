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
 * @note The SSIM does not depend on the order of traversal of the images, so you can safely swap the
 *       `step` and `stride` (as well as the `width` and `height`) parameters if this improves cache
 *       hit rates. As long as both images are traversed in the same order.
 *
 * @note Multi-threading support implemented via OpenMP (hence the name).
 *
 * @param [in] params The parameter structure
 *
 * @retval >=0 The image's SSIM, in the range [0;1].
 * @retval <0  An error occurred, call `get_errno()` to retrieve the error number.
 */
float compute_ssim_openmp(const UnthreadedParams& params) RMGR_NOEXCEPT;


}} // namespace rmgr::ssim

#endif // RMGR_SSIM_H
