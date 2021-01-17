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

project(rmgr-ssim
    VERSION   1.0.0
    LANGUAGES CXX
)

include(GNUInstallDirs)


###################################################################################################
# Options

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
    find_package(OpenMP)
    list(APPEND RMGR_SSIM_DEFINITIONS "RMGR_SSIM_USE_OPENMP=1")
else()
    list(APPEND RMGR_SSIM_DEFINITIONS "RMGR_SSIM_USE_OPENMP=0")
endif()


###################################################################################################
# Compiler-specific flags

if (CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID MATCHES ".*Clang"))
    list(APPEND RMGR_SSIM_COMPILE_OPTIONS "-Wall")

    include(CheckCXXSymbolExists)
    check_cxx_symbol_exists("__i386__"  "" RMGR_PREDEF_I386)
    check_cxx_symbol_exists("__amd64__" "" RMGR_PREDEF_AMD64)
    if (RMGR_PREDEF_I386 OR RMGR_PREDEF_AMD64)
        set(RMGR_SSIM_SSE_FLAGS    "-msse2")
        set(RMGR_SSIM_AVX_FLAGS    "-mavx")
        set(RMGR_SSIM_FMA_FLAGS    "-mfma")
        set(RMGR_SSIM_AVX512_FLAGS "-mavx512f")
        if (CMAKE_COMPILER_IS_GNUCXX AND (WIN32 OR CYGWIN))
            list(APPEND RMGR_SSIM_AVX512_FLAGS "-fno-exceptions" "-fno-asynchronous-unwind-tables") # Fixes a build error in AVX-512 code
        endif()
    else()
        check_cxx_symbol_exists("__aarch64__" "" RMGR_PREDEF_AARCH64)
        if (RMGR_PREDEF_AARCH64)
            set(RMGR_SSIM_NEON_FLAGS "")
        else()
            check_cxx_symbol_exists("__arm__"  ""    RMGR_PREDEF_ARM)
            set(RMGR_SSIM_NEON_FLAGS "-mfpu=neon")
            check_cxx_symbol_exists(__ARM_PCS_VFP "" RMGR_PREDEF_ARM_PCS_VFP)
            if (__ARM_PCS_VFP)
                set(RMGR_SSIM_ARM_COMPILE_OPTIONS "${RMGR_SSIM_NEON_FLAGS} -mfloat-abi=hard")
            else()
                set(RMGR_SSIM_ARM_COMPILE_OPTIONS "${RMGR_SSIM_NEON_FLAGS} -mfloat-abi=softfp")
            endif()
        endif()
    endif()

elseif (MSVC)
    list(APPEND RMGR_SSIM_COMPILE_OPTIONS "/W4")
    list(APPEND RMGR_SSIM_COMPILE_OPTIONS "/MP") # Multi-processor build

    include(CheckCXXSymbolExists)
    check_cxx_symbol_exists("_M_IX86"  "" RMGR_PREDEF_M_IX86)
    check_cxx_symbol_exists("_M_AMD64" "" RMGR_PREDEF_M_AMD64)
    if (RMGR_PREDEF_M_IX86 OR RMGR_PREDEF_M_AMD64)
        set(RMGR_SSIM_SSE_FLAGS    "/arch:SSE2")
        set(RMGR_SSIM_AVX_FLAGS    "/arch:AVX")
        set(RMGR_SSIM_FMA_FLAGS    "/arch:AVX2")
        set(RMGR_SSIM_AVX512_FLAGS "/arch:AVX512")
    else()
        check_cxx_symbol_exists("_M_ARM"   "" RMGR_PREDEF_M_ARM)
        check_cxx_symbol_exists("_M_ARM64" "" RMGR_PREDEF_M_ARM64)
        if (RMGR_PREDEF_M_ARM OR RMGR_PREDEF_M_ARM64)
            set(RMGR_SSIM_NEON_FLAGS "")
        endif()
    endif()
endif()


###################################################################################################
# Source files

set(RMGR_SSIM_FILES
    "include/rmgr/ssim.h"
    "src/ssim.cpp"
    "src/ssim_internal.h"
    "src/ssim_sse.cpp"
    "src/ssim_avx.cpp"
    "src/ssim_avx512.cpp"
    "src/ssim_fma.cpp"
    "src/ssim_neon.cpp"
)

if (RMGR_SSIM_USE_OPENMP AND OpenMP_CXX_FOUND)
    set_source_files_properties("src/ssim.cpp" PROPERTIES COMPILE_OPTIONS "${OpenMP_CXX_FLAGS}")
endif()

include(CheckCXXCompilerFlag)
foreach (is sse avx fma neon avx512)
    string(TOUPPER "${is}" ISU)
    if (RMGR_SSIM_${ISU}_FLAGS)
        check_cxx_compiler_flag("${RMGR_SSIM_${ISU}_FLAGS}" RMGR_SSIM_${ISU}_FLAGS_SUPPORTED)
        if (RMGR_SSIM_${ISU}_FLAGS_SUPPORTED)
            set_source_files_properties("src/ssim_${is}.cpp" PROPERTIES COMPILE_OPTIONS "${RMGR_SSIM_${ISU}_FLAGS}")
        endif()
    endif()
endforeach()

source_group("Source Files" FILES ${RMGR_SSIM_FILES})


###################################################################################################
# Library

add_library(rmgr-ssim STATIC ${RMGR_SSIM_FILES})

target_include_directories(rmgr-ssim
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    INTERFACE
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_FULL_INCLUDE_DIR}>"
)

#target_compile_features(rmgr-ssim PUBLIC cxx_std_11)
target_compile_options(rmgr-ssim PRIVATE ${RMGR_SSIM_COMPILE_OPTIONS})
target_compile_definitions(rmgr-ssim PRIVATE ${RMGR_SSIM_DEFINITIONS} _USE_MATH_DEFINES)

if (RMGR_SSIM_USE_OPENMP AND OpenMP_CXX_FOUND)
    set(RMGR_SSIM_OPENMP_FILES
        "include/rmgr/ssim-openmp.h"
        "src/ssim-openmp.cpp"
    )
    source_group("Source Files" FILES ${RMGR_SSIM_OPENMP_FILES})

    add_library(rmgr-ssim-openmp ${RMGR_SSIM_OPENMP_FILES})

    target_link_libraries(rmgr-ssim-openmp
        PUBLIC  rmgr-ssim
        PRIVATE OpenMP::OpenMP_CXX
    )
endif()


###################################################################################################
# Install rules

install(TARGETS rmgr-ssim ARCHIVE     DESTINATION "${CMAKE_INSTALL_LIBDIR}")
install(FILES   "include/rmgr/ssim.h" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/rmgr")
if (TARGET rmgr-ssim-openmp)
    install(FILES "include/rmgr/ssim-openmp.h" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/rmgr")
endif()

if (WIN32)
    set(CPACK_GENERATOR "ZIP")
else()
    set(CPACK_GENERATOR "TGZ")
endif()
include(CPack)


###################################################################################################
# Sub-projects

if (RMGR_SSIM_BUILD_TESTS)
    add_subdirectory(tests)
    set_directory_properties(PROPERTIES VS_STARTUP_PROJECT rmgr-ssim-tests)
endif()
