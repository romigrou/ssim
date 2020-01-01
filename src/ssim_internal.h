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

#ifndef RMGR_SSIM_SIMD_H
#define RMGR_SSIM_SIMD_H

#include <rmgr/ssim.h>

#ifndef RMGR_SSIM_USE_DOUBLE
    #define RMGR_SSIM_USE_DOUBLE  0
#endif

namespace rmgr { namespace ssim
{

#if RMGR_SSIM_USE_DOUBLE
    typedef double Float;
#else
    typedef float  Float;
#endif


typedef void (*MultiplyFct)(Float* product, const Float* a, const Float* b, uint32_t width, uint32_t height, size_t stride, uint32_t margin) RMGR_NOEXCEPT_TYPEDEF;

typedef void (*GaussianBlurFct)(Float* dest, ptrdiff_t destStride, const Float* srce, ptrdiff_t srceStride, int32_t width, int32_t height, const Float kernel[], int radius) RMGR_NOEXCEPT_TYPEDEF;

typedef double (*SumTileFct)(uint32_t tileWidth, uint32_t tileHeight, uint32_t tileStride, Float c1, Float c2,
                             const Float* muATile, const Float* muBTile, const Float* sigmaA2Tile, const Float* sigmaB2Tile, const Float* sigmaABTile,
                             float* ssimTile, ptrdiff_t ssimStep, ptrdiff_t ssimStride) RMGR_NOEXCEPT_TYPEDEF;


#if RMGR_ARCH_IS_X86_ANY
namespace sse
{
    extern const MultiplyFct     g_multiplyFct;
    extern const GaussianBlurFct g_gaussianBlurFct;
    extern const SumTileFct      g_sumTileFct;
}

namespace avx
{
    extern const MultiplyFct     g_multiplyFct;
    extern const GaussianBlurFct g_gaussianBlurFct;
    extern const SumTileFct      g_sumTileFct;
}

namespace fma
{
    using avx::g_multiplyFct;
    using avx::g_sumTileFct;
    extern const GaussianBlurFct g_gaussianBlurFct;
}
#endif


}} // namespace rmgr::ssim

#endif // RMGR_SSIM_H
