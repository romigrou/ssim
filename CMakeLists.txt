# Copyright (c) 2020, Romain Bailly
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
#
# 1. The origin of this software must not be misrepresented; you must not
#    claim that you wrote the original software. If you use this software
#    in a product, an acknowledgment in the product documentation would be
#    appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
#    misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

cmake_minimum_required(VERSION 3.8)

project(rmgr-ssim CXX)

option(RMGR_SSIM_USE_DOUBLE  "Whether to use double instead of float for increased precision (but slower)" OFF)
option(RMGR_SSIM_USE_OPENMP  "Whether to use OpenMP for multi-threaded processing"                         OFF)
option(RMGR_SSIM_BUILD_TESTS "Whether to build rmgr::ssim's unit tests"                                    OFF)

set(RMGR_SSIM_COMPILE_OPTIONS)
set(RMGR_SSIM_DEFINITIONS)

if (RMGR_SSIM_USE_DOUBLE)
    list(APPEND RMGR_SSIM_DEFINITIONS "RMGR_SSIM_USE_DOUBLE=1")
else()
    list(APPEND RMGR_SSIM_DEFINITIONS "RMGR_SSIM_USE_DOUBLE=0")
endif()

if (RMGR_SSIM_USE_OPENMP)
    list(APPEND RMGR_SSIM_DEFINITIONS "RMGR_SSIM_USE_OPENMP=1")
else()
    list(APPEND RMGR_SSIM_DEFINITIONS "RMGR_SSIM_USE_OPENMP=0")
endif()

# Compiler-specific flags
if (CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
    list(APPEND RMGR_SSIM_COMPILE_OPTIONS "-Wall")

    include(CheckCXXSymbolExists)
    check_cxx_symbol_exists("__i386__"  "" RMGR_PREDEF_I386)
    check_cxx_symbol_exists("__amd64__" "" RMGR_PREDEF_AMD64)
    if (RMGR_PREDEF_I386 OR RMGR_PREDEF_AMD64)
        set(RMGR_SSIM_SSE_FLAGS "-msse2")
        set(RMGR_SSIM_AVX_FLAGS "-mavx")
        set(RMGR_SSIM_FMA_FLAGS "-mfma")
    endif()
    if (RMGR_SSIM_USE_OPENMP)
        set(RMGR_SSIM_OPENMP_FLAGS "-fopenmp")
    endif()

elseif (MSVC)
    list(APPEND RMGR_SSIM_COMPILE_OPTIONS "/W4")
    list(APPEND RMGR_SSIM_COMPILE_OPTIONS "/MP") # Multi-processor build

    include(CheckCXXSymbolExists)
    check_cxx_symbol_exists("_M_IX86"  "" RMGR_PREDEF_M_IX86)
    check_cxx_symbol_exists("_M_AMD64" "" RMGR_PREDEF_M_AMD64)
    if (RMGR_PREDEF_M_IX86 OR RMGR_PREDEF_M_AMD64)
        set(RMGR_SSIM_SSE_FLAGS "/arch:SSE2")
        set(RMGR_SSIM_AVX_FLAGS "/arch:AVX")
        set(RMGR_SSIM_FMA_FLAGS "/arch:AVX")
    endif()
    if (RMGR_SSIM_USE_OPENMP)
        set(RMGR_SSIM_OPENMP_FLAGS "/openmp")
    endif()
endif()


set(RMGR_SSIM_FILES
    "include/rmgr/ssim.h"
    "src/ssim.cpp"
    "src/ssim_internal.h"
    "src/ssim_sse.cpp"
    "src/ssim_avx.cpp"
)

set_source_files_properties("src/ssim.cpp" PROPERTIES COMPILE_OPTIONS "${RMGR_SSIM_OPENMP_FLAGS}")

foreach (is sse avx)
    string(TOUPPER "${is}" ISU)
    if (DEFINED RMGR_SSIM_${ISU}_FLAGS)
        set_source_files_properties("src/ssim_${is}.cpp" PROPERTIES COMPILE_OPTIONS "${RMGR_SSIM_${ISU}_FLAGS}")
    else()
        set_source_files_properties("src/ssim_${is}.cpp" PROPERTIES HEADER_FILE_ONLY TRUE)
    endif()
endforeach()
source_group("Source Files" FILES ${RMGR_SSIM_FILES})


add_library(rmgr-ssim STATIC ${RMGR_SSIM_FILES})

target_include_directories(rmgr-ssim PUBLIC "include")
#target_compile_features(rmgr-ssim PUBLIC cxx_std_11)
target_compile_options(rmgr-ssim PRIVATE ${RMGR_SSIM_COMPILE_OPTIONS})
target_compile_definitions(rmgr-ssim PRIVATE ${RMGR_SSIM_DEFINITIONS} _USE_MATH_DEFINES)

if (RMGR_SSIM_BUILD_TESTS)
    add_subdirectory(tests)
    set_directory_properties(PROPERTIES VS_STARTUP_PROJECT rmgr-ssim-tests)
endif()
