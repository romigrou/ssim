cmake_minimum_required(VERSION 3.8)

project(rmgr-ssim CXX)

option(RMGR_SSIM_BUILD_TESTS "Whether to build rmgr::ssim's unit tests" OFF)

set(RMGR_SSIM_COMPILE_OPTIONS)

if (CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
    list(APPEND RMGR_SSIM_COMPILE_OPTIONS "-Wall")

    include(CheckCXXSymbolExists)
    check_cxx_symbol_exists("__i386__"  "" RMGR_PREDEF_I386)
    check_cxx_symbol_exists("__amd64__" "" RMGR_PREDEF_AMD64)
    if (RMGR_PREDEF_I386 OR RMGR_PREDEF_AMD64)
        set(RMGR_SSIM_SSE_FLAGS "-msse")
        set(RMGR_SSIM_AVX_FLAGS "-mavx")
        set(RMGR_SSIM_FMA_FLAGS "-mfma")
    endif()

elseif (MSVC)
    list(APPEND RMGR_SSIM_COMPILE_OPTIONS "/W4")
    list(APPEND RMGR_SSIM_COMPILE_OPTIONS "/MP") # Multi-processor build

    include(CheckCXXSymbolExists)
    check_cxx_symbol_exists("_M_IX86"  "" RMGR_PREDEF_M_IX86)
    check_cxx_symbol_exists("_M_AMD64" "" RMGR_PREDEF_M_AMD64)
    if (RMGR_PREDEF_M_IX86 OR RMGR_PREDEF_M_AMD64)
        set(RMGR_SSIM_SSE_FLAGS "/arch:SSE")
        set(RMGR_SSIM_AVX_FLAGS "/arch:AVX")
        set(RMGR_SSIM_FMA_FLAGS "/arch:AVX")
    endif()
endif()


set(RMGR_SSIM_FILES
    "include/rmgr/ssim.h"
    "src/ssim.cpp"
    "src/ssim_internal.h"
    "src/ssim_sse.cpp"
    "src/ssim_avx.cpp"
)

foreach (is sse avx)
    string(TOUPPER "${is}" ISU)
    if (DEFINED RMGR_SSIM_${ISU}_FLAGS)
        set_source_files_properties("src/ssim_${is}.cpp" PROPERTIES COMPILE_OPTIONS "${RMGR_SSIM_${ISU}_FLAGS}")
    else()
        set_source_files_properties("src/ssim_${is}.cpp" PROPERTIES HEADER_FILE_ONLY TRUE)
    endif()
endforeach()
source_group("Source Files" FILES ${RMGR_SSIM_FILES})

if (MSVC)
    list(APPEND RMGR_SSIM_FILES "src/rmgr_ssim.natvis")
endif()

add_library(rmgr-ssim STATIC ${RMGR_SSIM_FILES})

target_include_directories(rmgr-ssim PUBLIC "include")
#target_compile_features(rmgr-ssim PUBLIC cxx_std_11)
target_compile_options(rmgr-ssim PRIVATE ${RMGR_SSIM_COMPILE_OPTIONS})
target_compile_definitions(rmgr-ssim PRIVATE _USE_MATH_DEFINES)

if (RMGR_SSIM_BUILD_TESTS)
    add_subdirectory(tests)
    set_directory_properties(PROPERTIES VS_STARTUP_PROJECT rmgr-ssim-tests)
endif()
