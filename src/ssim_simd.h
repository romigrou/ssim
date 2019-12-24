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

#ifndef RMGR_SSIM_SIMD_H
#define RMGR_SSIM_SIMD_H

#include <rmgr/ssim.h>

namespace rmgr { namespace ssim
{

typedef void (*GaussianBlurFct)(float* dest, ptrdiff_t destStride, const float* srce, ptrdiff_t srceStride, int32_t width, int32_t height, const float kernel[], int radius) RMGR_NOEXCEPT_TYPEDEF;

namespace avx { extern const GaussianBlurFct g_gaussianBlurFct;}
namespace sse { extern const GaussianBlurFct g_gaussianBlurFct;}


}} // namespace rmgr::ssim

#endif // RMGR_SSIM_H
