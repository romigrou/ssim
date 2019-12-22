cmake_minimum_required(VERSION 3.8)

project(rmgr-ssim CXX)

option(RMGR_SSIM_BUILD_TESTS "Whether to build rmgr::ssim's unit tests" OFF)

set(RMGR_SSIM_COMPILE_OPTIONS)
if (CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
    list(APPEND RMGR_SSIM_COMPILE_OPTIONS "-Wall")
elseif (MSVC)
    list(APPEND RMGR_SSIM_COMPILE_OPTIONS "/W4")
    list(APPEND RMGR_SSIM_COMPILE_OPTIONS "/MP") # Multi-processor build
endif()


set(RMGR_SSIM_FILES
    "include/rmgr/ssim.h"
    "src/ssim.cpp"
)

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
