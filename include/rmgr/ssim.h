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

#ifndef RMGR_SSIM_H
#define RMGR_SSIM_H


#include <cassert>
#include <cstddef>


//=================================================================================================
// Compiler detection

/** @cond cecfb4eb5863e49f4cf59adc01ed0452 */
#ifndef RMGR_COMPILER_IS_NOT_DOXYGEN
    #define RMGR_COMPILER_IS_NOT_DOXYGEN  1
#endif
/** @endcond */

#if !defined(RMGR_COMPILER_IS_NOT_DOXYGEN)
    #define RMGR_COMPILER_IS_DOXYGEN                     1

#elif defined(__clang__)
    #ifndef RMGR_COMPILER_IS_CLANG
        #define RMGR_COMPILER_IS_CLANG                   1
    #endif
    #ifndef RMGR_COMPILER_VERSION_MAJOR
        #define RMGR_COMPILER_VERSION_MAJOR              __clang_major__
    #endif
    #ifndef RMGR_COMPILER_VERSION_MINOR
        #define RMGR_COMPILER_VERSION_MINOR              __clang_minor__
    #endif
    #ifndef RMGR_COMPILER_VERSION_PATCH
        #define RMGR_COMPILER_VERSION_PATCH              __clang_patchlevel__
    #endif
    #ifndef RMGR_WARNING_PUSH
        #define RMGR_WARNING_PUSH()                      _Pragma("clang diagnostic push")
    #endif
    #ifndef RMGR_WARNING_POP
        #define RMGR_WARNING_POP()                       _Pragma("clang diagnostic pop")
    #endif
    #ifndef RMGR_WARNING_CLANG_DISABLE
        #define RMGR_WARNING_CLANG_DO_DISABLE(string)    _Pragma(#string)
        #define RMGR_WARNING_CLANG_DISABLE(name)         RMGR_WARNING_CLANG_DO_DISABLE(clang diagnostic ignored name)
    #endif
    #ifndef RMGR_ALIGNED_VAR
        #define RMGR_ALIGNED_VAR(alignment, type, name)  type name __attribute__((aligned(alignment)))
    #endif

#elif defined(_MSC_VER)
    #ifndef RMGR_COMPILER_IS_MSVC
        #define RMGR_COMPILER_IS_MSVC                    1
    #endif
    #ifndef RMGR_COMPILER_VERSION_MAJOR
        #define RMGR_COMPILER_VERSION_MAJOR              (_MSC_VER / 100)
    #endif
    #ifndef RMGR_COMPILER_VERSION_MINOR
        #define RMGR_COMPILER_VERSION_MINOR              (_MSC_VER % 100)
    #endif
    #ifndef RMGR_COMPILER_VERSION_PATCH
        #define RMGR_COMPILER_VERSION_PATCH              (_MSC_FULL_VER % 100000)
    #endif
    #ifndef RMGR_WARNING_PUSH
        #define RMGR_WARNING_PUSH()                      __pragma(warning(push))
    #endif
    #ifndef RMGR_WARNING_POP
        #define RMGR_WARNING_POP()                       __pragma(warning(pop))
    #endif
    #ifndef RMGR_WARNING_MSVC_DISABLE
        #define RMGR_WARNING_MSVC_DISABLE(number)        __pragma(warning(disable: number))
    #endif
    #ifndef RMGR_ALIGNED_VAR
        #define RMGR_ALIGNED_VAR(alignment, type, name)  __declspec(align(alignment)) type name
    #endif

#elif defined(__GNUC__)
    #ifndef RMGR_COMPILER_IS_GCC
        #define RMGR_COMPILER_IS_GCC                     1
    #endif
    #ifndef RMGR_COMPILER_VERSION_MAJOR
        #define RMGR_COMPILER_VERSION_MAJOR              __GNUC__
    #endif
    #ifndef RMGR_COMPILER_VERSION_MINOR
        #define RMGR_COMPILER_VERSION_MINOR              __GNUC_MINOR__
    #endif
    #ifndef RMGR_COMPILER_VERSION_PATCH
        #define RMGR_COMPILER_VERSION_PATCH              __GNUC_PATCHLEVEL__
    #endif
    #ifndef RMGR_WARNING_PUSH
        #define RMGR_WARNING_PUSH()                      _Pragma("GCC diagnostic push")
    #endif
    #ifndef RMGR_WARNING_POP
        #define RMGR_WARNING_POP()                       _Pragma("GCC diagnostic pop")
    #endif
    #ifndef RMGR_WARNING_GCC_DISABLE
        #define RMGR_WARNING_GCC_DO_DISABLE(string)      _Pragma(#string)
        #define RMGR_WARNING_GCC_DISABLE(name)           RMGR_WARNING_GCC_DO_DISABLE(GCC diagnostic ignored name)
    #endif
    #ifndef RMGR_ALIGNED_VAR
        #define RMGR_ALIGNED_VAR(alignment, type, name)  type name __attribute__((aligned(alignment)))
    #endif
#endif

#ifndef RMGR_COMPILER_IS_DOXYGEN
    #define RMGR_COMPILER_IS_DOXYGEN       0 ///< Whether the compiler is Doxygen
#endif
#ifndef RMGR_COMPILER_IS_CLANG
    #define RMGR_COMPILER_IS_CLANG         0 ///< Whether the compiler is Clang
#endif
#ifndef RMGR_COMPILER_IS_MSVC
    #define RMGR_COMPILER_IS_MSVC          0 ///< Whether the compiler is Microsoft Visual C++
#endif
#ifndef RMGR_COMPILER_IS_GCC
    #define RMGR_COMPILER_IS_GCC           0 ///< Whether the compiler is GCC
#endif
#ifndef RMGR_COMPILER_IS_GCC_OR_CLANG
    #define RMGR_COMPILER_IS_GCC_OR_CLANG  (RMGR_COMPILER_IS_GCC || RMGR_COMPILER_IS_CLANG)
#endif
#ifndef RMGR_COMPILER_VERSION_IS_AT_LEAST
    /// Returns non-zero if the compiler version is >= the specified one
    #define RMGR_COMPILER_VERSION_IS_AT_LEAST(major,minor,patch)                                   \
        (    (major) <  RMGR_COMPILER_VERSION_MAJOR                                                \
         || ((major) == RMGR_COMPILER_VERSION_MAJOR && (    (minor) <  RMGR_COMPILER_VERSION_MINOR \
                                                        || ((minor) == RMGR_COMPILER_VERSION_MINOR && (patch)<=RMGR_COMPILER_VERSION_PATCH))))
#endif
#ifndef RMGR_COMPILER_IS_CLANG_AT_LEAST
    #define RMGR_COMPILER_IS_CLANG_AT_LEAST(major,minor,patch)   (RMGR_COMPILER_IS_CLANG && RMGR_COMPILER_VERSION_IS_AT_LEAST((major),(minor),(patch)))
#endif
#ifndef RMGR_COMPILER_IS_MSVC_AT_LEAST
    #define RMGR_COMPILER_IS_MSVC_AT_LEAST(major,minor,patch)    (RMGR_COMPILER_IS_MSVC  && RMGR_COMPILER_VERSION_IS_AT_LEAST((major),(minor),(patch)))
#endif
#ifndef RMGR_COMPILER_IS_GCC_AT_LEAST
    #define RMGR_COMPILER_IS_GCC_AT_LEAST(major,minor,patch)     (RMGR_COMPILER_IS_GCC   && RMGR_COMPILER_VERSION_IS_AT_LEAST((major),(minor),(patch)))
#endif
#ifndef RMGR_COMPILER_IS_CLANG_LESS_THAN
    #define RMGR_COMPILER_IS_CLANG_LESS_THAN(major,minor,patch)  (RMGR_COMPILER_IS_CLANG && !RMGR_COMPILER_VERSION_IS_AT_LEAST((major),(minor),(patch)))
#endif
#ifndef RMGR_COMPILER_IS_MSVC_LESS_THAN
    #define RMGR_COMPILER_IS_MSVC_LESS_THAN(major,minor,patch)   (RMGR_COMPILER_IS_MSVC  && !RMGR_COMPILER_VERSION_IS_AT_LEAST((major),(minor),(patch)))
#endif
#ifndef RMGR_COMPILER_IS_GCC_LESS_THAN
    #define RMGR_COMPILER_IS_GCC_LESS_THAN(major,minor,patch)    (RMGR_COMPILER_IS_GCC   && !RMGR_COMPILER_VERSION_IS_AT_LEAST((major),(minor),(patch)))
#endif
#ifndef RMGR_WARNING_PUSH
    #define RMGR_WARNING_PUSH()
#endif
#ifndef RMGR_WARNING_POP
    #define RMGR_WARNING_POP()
#endif
#ifndef RMGR_WARNING_MSVC_DISABLE
    #define RMGR_WARNING_MSVC_DISABLE(number)
#endif
#ifndef RMGR_WARNING_GCC_DISABLE
    #define RMGR_WARNING_GCC_DISABLE(name)
#endif
#ifndef RMGR_WARNING_CLANG_DISABLE
    #define RMGR_WARNING_CLANG_DISABLE(name)
#endif


//=================================================================================================
// Architecture detection

/**
 * @brief Whether the architecture is 32-bit x86 (exclusive of the x32 ABI)
 *
 * @see `#RMGR_ARCH_IS_X86_64`
 * @see `#RMGR_ARCH_IS_X86_ANY`
 */
#ifndef RMGR_ARCH_IS_X86_32
    #if defined(_M_IX86) || defined(__i386__)
        #define RMGR_ARCH_IS_X86_32  1
    #else
        #define RMGR_ARCH_IS_X86_32  0
    #endif
#endif

/**
 * @brief Whether the architecture is 64-bit x86 (inclusive of the x32 ABI)
 *
 * @see `#RMGR_ARCH_IS_X86_32`
 * @see `#RMGR_ARCH_IS_X86_ANY`
 */
#ifndef RMGR_ARCH_IS_X86_64
    #if defined(_M_X64) || defined(_M_AMD64) || defined(__amd64__)
        #define RMGR_ARCH_IS_X86_64  1
    #else
        #define RMGR_ARCH_IS_X86_64  0
    #endif
#endif

/**
 * @brief Whether the architecture is x86, in either 32-bit or 64-bit flavour
 *
 * @see `#RMGR_ARCH_IS_X86_32`
 * @see `#RMGR_ARCH_IS_X86_64`
 */
#ifndef RMGR_ARCH_IS_X86_ANY
    #define RMGR_ARCH_IS_X86_ANY  (RMGR_ARCH_IS_X86_32 || RMGR_ARCH_IS_X86_64)
#endif

/**
 * @brief Whether the architecture is 32-bit ARM
 *
 * @see `#RMGR_ARCH_IS_ARM_64`
 * @see `#RMGR_ARCH_IS_ARM_ANY`
 */
#ifndef RMGR_ARCH_IS_ARM_32
    #if defined(__arm__) || defined(_M_ARM)
        #define RMGR_ARCH_IS_ARM_32  1
    #else
        #define RMGR_ARCH_IS_ARM_32  0
    #endif
#endif

/**
 * @brief Whether the architecture is 64-bit ARM
 *
 * @see `#RMGR_ARCH_IS_ARM_32`
 * @see `#RMGR_ARCH_IS_ARM_ANY`
 */
#ifndef RMGR_ARCH_IS_ARM_64
    #if defined(__aarch64__) || defined(_M_ARM64)
        #define RMGR_ARCH_IS_ARM_64  1
    #else
        #define RMGR_ARCH_IS_ARM_64  0
    #endif
#endif

/**
 * @brief Whether the architecture is ARM, in either 32-bit or 64-bit flavour
 *
 * @see `#RMGR_ARCH_IS_ARM_32`
 * @see `#RMGR_ARCH_IS_ARM_64`
 */
#ifndef RMGR_ARCH_IS_ARM_ANY
    #define RMGR_ARCH_IS_ARM_ANY  (RMGR_ARCH_IS_ARM_32 || RMGR_ARCH_IS_ARM_64)
#endif

/**
 * @brief Whether the architecture is little endian
 *
 * @note In some exotic cases (PDP), the architecture may be neither little nor big endian.
 *
 * @see `#RMGR_ARCH_IS_BIG_ENDIAN`
 */
#ifndef RMGR_ARCH_IS_LITTLE_ENDIAN
    #if defined(RMGR_ARCH_IS_BIG_ENDIAN) && RMGR_ARCH_IS_BIG_ENDIAN
        #define RMGR_ARCH_IS_LITTLE_ENDIAN  0
    #elif defined(__BYTE_ORDER__) && __BYTE_ORDER__==__ORDER_LITTLE_ENDIAN__
        #define RMGR_ARCH_IS_LITTLE_ENDIAN  1
    #elif RMGR_ARCH_IS_X86_ANY
        #define RMGR_ARCH_IS_LITTLE_ENDIAN  1 // x86 is always little endian
    #elif defined(_WIN32)
        #define RMGR_ARCH_IS_LITTLE_ENDIAN  1 // Windows is always little endian (as of now at least)
    #else
        #define RMGR_ARCH_IS_LITTLE_ENDIAN  0
    #endif
#endif

/**
 * @brief Whether the architecture is big endian
 *
 * @note In some exotic cases (PDP), the architecture may be neither little nor big endian.
 *
 * @see `#RMGR_ARCH_IS_LITTLE_ENDIAN`
 */
#ifndef RMGR_ARCH_IS_BIG_ENDIAN
    #if defined(RMGR_ARCH_IS_LITTLE_ENDIAN) && RMGR_ARCH_IS_LITTLE_ENDIAN
        #define RMGR_ARCH_IS_BIG_ENDIAN  0
    #elif defined(__BYTE_ORDER__) && __BYTE_ORDER__==__ORDER_BIG_ENDIAN__
        #define RMGR_ARCH_IS_BIG_ENDIAN  1
    #else
        #define RMGR_ARCH_IS_BIG_ENDIAN  0
    #endif
#endif

#if RMGR_ARCH_IS_LITTLE_ENDIAN && RMGR_ARCH_IS_BIG_ENDIAN
    #error Architecture cannot be both little and big endian
#endif

//=================================================================================================
// C++ version

#ifndef RMGR_CPP_VERSION
    #ifdef _MSVC_LANG
        #define RMGR_CPP_VERSION  _MSVC_LANG
    #else
        #define RMGR_CPP_VERSION  __cplusplus
    #endif
#endif

#ifndef RMGR_NOEXCEPT
    #if RMGR_CPP_VERSION >= 201103
        #define RMGR_NOEXCEPT  noexcept
    #else
        #define RMGR_NOEXCEPT
    #endif
#endif

#ifndef RMGR_NOEXCEPT_TYPEDEF
    #if RMGR_CPP_VERSION >= 201703
        #define RMGR_NOEXCEPT_TYPEDEF  RMGR_NOEXCEPT
    #else
        #define RMGR_NOEXCEPT_TYPEDEF
    #endif
#endif


//=================================================================================================
// Inlining control

#ifndef RMGR_FORCEINLINE
    #if RMGR_COMPILER_IS_MSVC
        #define RMGR_FORCEINLINE  __forceinline
    #elif RMGR_COMPILER_IS_GCC_OR_CLANG
        #define RMGR_FORCEINLINE  inline __attribute__((always_inline))
    #else
        #define RMGR_FORCEINLINE  inline
    #endif
#endif


#ifndef RMGR_NOINLINE
    #if RMGR_COMPILER_IS_MSVC
        #define RMGR_NOINLINE  __declspec(noinline)
    #elif RMGR_COMPILER_IS_GCC_OR_CLANG
        #define RMGR_NOINLINE  __attribute__((noinline))
    #else
        #define RMGR_NOINLINE
    #endif
#endif


//=================================================================================================
// Other stuff

#ifndef RMGR_ALIGNED_VAR
    #if RMGR_CPP_VERSION >= 201103
        #define RMGR_ALIGNED_VAR(alignment, type, name)  alignas(alignment) type name
    #else
        #define RMGR_ALIGNED(alignment, type, name)      type name
    #endif
#endif

#ifndef RMGR_COMPILER_SUPPORTS_ARM_NEON
    #if (RMGR_ARCH_IS_ARM_64 && !RMGR_COMPILER_IS_MSVC) || (RMGR_ARCH_IS_ARM_32 && (defined(__ARM_NEON) || RMGR_COMPILER_IS_MSVC))
        #define RMGR_COMPILER_SUPPORTS_ARM_NEON  1
    #else
        #define RMGR_COMPILER_SUPPORTS_ARM_NEON  0
    #endif
#endif

/**
 * @namespace rmgr
 * @brief The root namespace for all my projects
 */

/**
 * @namespace rmgr::ssim
 * @brief Namespace that contains all things related to SSIM
 */


// Workaround for MSVC prior to 2010 lacking stdint.h
#if !RMGR_COMPILER_IS_DOXYGEN
#if RMGR_COMPILER_IS_MSVC_LESS_THAN(16,0,0)
namespace rmgr { namespace ssim
{
    typedef unsigned __int8  uint8_t;
    typedef signed   __int32 int32_t;
    typedef unsigned __int32 uint32_t;
    typedef unsigned __int64 uint64_t;
    const uint8_t UINT8_MAX = 255;
}}
#else
#include <cstdint>
namespace rmgr { namespace ssim
{
    typedef ::uint8_t  uint8_t;
    typedef ::int32_t  int32_t;
    typedef ::uint32_t uint32_t;
    typedef ::uint64_t uint64_t;
}}
#endif
#endif


namespace rmgr { namespace ssim
{


typedef void* (*AllocFct)(size_t size, size_t alignment) RMGR_NOEXCEPT_TYPEDEF;
typedef void  (*DeallocFct)(void* address) RMGR_NOEXCEPT_TYPEDEF;


/**
 * @brief Prototype for the function to be called from inside a thread pool
 *
 * @param [in] arg    The per-thread argument as passed to the thread pool
 * @param [in] jobNum The current job's number. Must be < `jobCount` as passed to `ThreadPoolFct`.
 */
typedef void (*ThreadFct)(void* arg, unsigned jobNum) RMGR_NOEXCEPT_TYPEDEF;


/**
 * @brief Prototype for a thread pool implementation
 *
 * @param [in] fct         The function to run (all threads run the same function).
 * @param [in] args        The argument to `fct` for each of the threads. There are `threadCount` entries in this array.<br>
 *                         Entries can be used in any order as long as an entry is used by only one thread at a time.
 * @param [in] threadCount How many concurrent threads can be used at the maximum.<br>
 *                         This value is non-zero and is &le; to the value passed to `compute_ssim()`.
 * @param [in] jobCount    How many times `fct` must be called in total.
 * @param [in] context     The value passed in to `compute_ssim` as `threadPoolContext`.
 *                         This is a user-defined value that can contain whatever.
 *
 * @retval 0     All went fine
 * @retval Other An error occurred
 */
typedef int (*ThreadPoolFct)(void* context, ThreadFct fct, void* const args[], unsigned threadCount, unsigned jobCount) RMGR_NOEXCEPT_TYPEDEF;


/**
 * @brief Set of parameters relative to an input image
 *
 * This structure can handle almost any kind of image storage scheme (interleaved or planar, top-down or
 * bottom-up, row-major or column-major, ...). All you need is to specify the `step` and `stride` parameters
 * such that the address of a pixel's channel is given by `topLeft + x * imgStep + y * imgStride` with `x` and
 * `y` being the horizontal and vertical coordinates, respectively.
 *
 * Convenience functions to initialize the parameters for the two most common schemes are provided,
 * but you can (and should) bypass them if you need more flexibility.
 */
struct ImgParams
{
    const uint8_t* topLeft; ///< Pointer to the considered channel of the image's top-left pixel
    ptrdiff_t      step;    ///< The distance (in bytes) between a pixel and the one immediately to its right
    ptrdiff_t      stride;  ///< The distance (in bytes) between a pixel and the one immediately below it

    /**
     * @brief Sets the parameters for an interleaved image
     *
     * @param [in] data         Pointer to the image's top-left pixel
     * @param [in] imgStride    The distance (in bytes) between a pixel and the one immediately below it.
     *                          This is negative for bottom-up images.
     * @param [in] channelNum   The channel's whose SSIM is to be computed
     * @param [in] channelCount How many channels the image contains
     */
    void init_interleaved(const uint8_t* data, ptrdiff_t imgStride, unsigned channelCount, unsigned channelNum) RMGR_NOEXCEPT
    {
        assert(data != NULL);
        assert(channelNum < channelCount);
        topLeft = data + channelNum;
        step    = channelCount;
        stride  = imgStride;
    }

    /**
     * @brief Sets the parameters for an interleaved image (assuming one channel per plane)
     *
     * @param [in] planes   For each plane, a pointer to its top-left pixel
     * @param [in] strides  For each plane, the distance (in bytes) between a pixel and the one immediately below it.
     *                      This is negative for bottom-up images.
     * @param [in] planeNum The channel's whose SSIM is to be computed
     */
    void init_planar(uint8_t const* const planes[], const ptrdiff_t strides[], unsigned planeNum) RMGR_NOEXCEPT
    {
        assert(planes != NULL && planes[planeNum] != NULL);
        assert(strides != NULL);
        topLeft = planes[planeNum];
        step    = 1;
        stride  = strides[planeNum];
    }
};


/**
 * @brief Set of parameters, except those related to threading
 */
struct UnthreadedParams
{
    // Input
    uint32_t       width;      ///< The images' width,  in pixels
    uint32_t       height;     ///< The images' height, in pixels
    ImgParams      imgA;       ///< The first  image's parameters
    ImgParams      imgB;       ///< The second image's parameters

    // SSIM map
    float*         ssimMap;    ///< A pointer to the top-left pixel the SSIM map. You can set this to `NULL` if you don't need a map
    ptrdiff_t      ssimStep;   ///< The distance (in number of `float`) between a pixel and the one immediately to its right
    ptrdiff_t      ssimStride; ///< The distance (in number of `float`) between a pixel and the one immediately below it (negative for bottom-up)

    // Allocation
    AllocFct       alloc;      ///< The allocation function. If `NULL`, stack storage is used.
    DeallocFct     dealloc;    ///< The deallocation function. Ignored if `alloc` is `NULL`.

    /**
     * @brief Sets the allocation functions to use `malloc()` and `free()`
     */
    void use_default_allocator() RMGR_NOEXCEPT;
};


/**
 * @brief Full set of parameters
 */
struct Params: public UnthreadedParams
{
    ThreadPoolFct  threadPool;        ///< A function that dispatches jobs on a thread pool. If `NULL` the processing is done in mono-thread mode.
    void*          threadPoolContext; ///< A user-defined value that will be passed as-is to the thread pool. Ignored if `threadPool` is `NULL`.
    unsigned       threadCount;       ///< How many threads are present in the thread pool. Ignored if `threadPool` is `NULL`.
};


/**
 * @brief Computes the SSIM of a single channel of two images and, optionally, the per-pixel SSIM map
 *
 * @note The SSIM does not depend on the order of traversal of the images, so you can safely swap the
 *       `step` and `stride` (as well as the `width` and `height`) parameters if this improves cache
 *       hit rates. As long as both images are traversed in the same order.
 *
 * @retval >=0 The image's SSIM, in the range [0;1].
 * @retval <0  An error occurred, call `get_errno()` to retrieve the error number.
 */
float compute_ssim(const Params& params) RMGR_NOEXCEPT;


/**
 * @brief Extracts the error number from an SSIM value
 *
 * @retval 0     No error occurred
 * @retval Other One of the error numbers from `cerrno`
 */
inline int get_errno(float ssim) RMGR_NOEXCEPT
{
    return (ssim>=0) ? 0 : -int(ssim);
}


}} // namespace rmgr::ssim

#endif // RMGR_SSIM_H
