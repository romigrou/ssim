/*
 * Copyright (c) 2023, Romain Bailly
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
#include <assert.h>
#include <omp.h>


static int run_in_openmp(void* context, rmgr_ssim_ThreadFct fct, void* const args[], rmgr_uint32_t threadCount, rmgr_uint32_t jobCount) RMGR_NOEXCEPT
{
    int jobNum;
    #pragma omp parallel for num_threads(threadCount)
    for (jobNum=0; jobNum < (int)jobCount; ++jobNum)
    {
        const unsigned threadNum = omp_get_thread_num();
        assert(threadNum < threadCount);
        fct(args[threadNum], jobNum);
    }
    return 0;
}


int rmgr_ssim_compute_ssim_openmp(float* ssim, const rmgr_ssim_Params* params) RMGR_NOEXCEPT
{
    rmgr_ssim_ThreadPool threadPool;
    threadPool.dispatch    = run_in_openmp;
    threadPool.context     = NULL;
    threadPool.threadCount = omp_get_num_procs();
    return rmgr_ssim_compute_ssim(ssim, params, &threadPool);
}
