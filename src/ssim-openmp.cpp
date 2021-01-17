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

#include <rmgr/ssim-openmp.h>
#include <cassert>
#include <omp.h>


namespace rmgr { namespace ssim
{


static int run_in_openmp(ThreadFct fct, void* const args[], unsigned threadCount, unsigned jobCount)
{
    #pragma omp parallel for num_threads(threadCount)
    for (int jobNum=0; jobNum < int(jobCount); ++jobNum)
    {
        const unsigned threadNum = omp_get_thread_num();
        assert(threadNum < threadCount);
        fct(args[threadNum], jobNum);
    }
    return 0;
}


float compute_ssim_openmp(uint32_t width, uint32_t height,
                       const uint8_t* imgAData, ptrdiff_t imgAStep, ptrdiff_t imgAStride,
                       const uint8_t* imgBData, ptrdiff_t imgBStep, ptrdiff_t imgBStride,
                       float* ssimMap, ptrdiff_t ssimStep, ptrdiff_t ssimStride, unsigned flags) RMGR_NOEXCEPT
{
    const unsigned threadCount = omp_get_num_procs();
    return compute_ssim(width, height, imgAData, imgAStep, imgAStride, imgBData, imgBStep, imgBStride, ssimMap, ssimStep, ssimStride, run_in_openmp, threadCount, flags);
}


}} // namespace rmgr::ssim
