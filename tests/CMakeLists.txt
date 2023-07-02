# Copyright (c) 2023, Romain Bailly
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

function(rmgr_download_and_extract url path createFolder)
    # Create destination folder if needed
    get_filename_component(folder "${path}" DIRECTORY)
    if (NOT EXISTS "${folder}")
        file(MAKE_DIRECTORY "${folder}")
    endif()

    # Download
    get_filename_component(filename "${path}" NAME)
    if (NOT EXISTS "${path}")
        message(STATUS "Downloading ${filename}")
        file(DOWNLOAD "${url}" "${path}" STATUS downloadStatus)
        list(GET downloadStatus 0 downloadStatusValue)
        if (NOT downloadStatusValue EQUAL 0)
            list(GET downloadStatus 1 downloadStatusMsg)
            message(FATAL_ERROR "Failed to download ${filename}: ${downloadStatusMsg}")
            if (EXISTS "${path}")
                file(REMOVE "${path}")
            endif()
        endif()
    endif()

    # Extract
    if (createFolder)
        get_filename_component(name "${filename}" NAME_WE)
        set(folder "${folder}/${name}")
        if (NOT EXISTS "${folder}")
            file(MAKE_DIRECTORY "${folder}")
        endif()
    endif()
    message(STATUS "Extracting ${filename}")
    execute_process(COMMAND           "${CMAKE_COMMAND}" -E tar xf "${path}"
                    WORKING_DIRECTORY "${folder}"
                    RESULT_VARIABLE   extractResult
                    ERROR_VARIABLE    extractError)
    if (NOT extractResult EQUAL 0)
        message(FATAL_ERROR "Failed to extract ${filename}: ${extractError}")
    endif()
endfunction()


################################################################################
# GTest

if (TARGET gtest)
    set(GTEST_LIBRARIES    gtest)
    set(GTEST_INCLUDE_DIRS "${gtest_SOURCE_DIR}/include")
else()
    find_package(GTest)
    if (NOT GTEST_FOUND)
        set(GTEST_VERSION 1.12.1)
        set(GTEST_DIR     "${CMAKE_CURRENT_SOURCE_DIR}/gtest/googletest-release-${GTEST_VERSION}")
        set(GTEST_URL     "https://github.com/google/googletest/archive/release-${GTEST_VERSION}.tar.gz")

        if (NOT EXISTS "${GTEST_DIR}")
            rmgr_download_and_extract("${GTEST_URL}" "${GTEST_DIR}.tag.gz" FALSE)
        endif()

        if (MSVC)
            set(gtest_force_shared_crt ON CACHE BOOL "")
        endif()

        add_subdirectory("${GTEST_DIR}" EXCLUDE_FROM_ALL)
        set(GTEST_LIBRARIES    gtest)
        set(GTEST_INCLUDE_DIRS "${gtest_SOURCE_DIR}/include")
    endif()
endif()


###############################################################################
# Main Target

get_filename_component(RMGR_SSIM_TESTS_IMAGES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/images" ABSOLUTE)

set(RMGR_SSIM_TESTS_FILES
    "rmgr-ssim-tests.cpp"
    "ssim_naive.h"
    "../src/stb_image.h"
)

source_group("Source Files" FILES ${RMGR_SSIM_TESTS_FILES})

add_executable(rmgr-ssim-tests EXCLUDE_FROM_ALL ${RMGR_SSIM_TESTS_FILES})

target_link_libraries(rmgr-ssim-tests PRIVATE
    ${GTEST_LIBRARIES}
    rmgr-ssim
)

if (TARGET rmgr-ssim-openmp)
    target_link_libraries(rmgr-ssim-tests PRIVATE rmgr-ssim-openmp)
    target_compile_definitions(rmgr-ssim-tests PRIVATE "RMGR_SSIM_USE_OPENMP=1")
endif()


target_include_directories(rmgr-ssim-tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_compile_options(rmgr-ssim-tests PRIVATE ${RMGR_SSIM_COMPILE_OPTIONS})
target_compile_definitions(rmgr-ssim-tests PRIVATE "RMGR_SSIM_TESTS_IMAGES_DIR=\"${RMGR_SSIM_TESTS_IMAGES_DIR}\"" ${RMGR_SSIM_DEFINITIONS} _USE_MATH_DEFINES)

if (MSVC)
    # Silence warnings
    target_compile_options(rmgr-ssim-tests PRIVATE "/wd4530") # C++ exception handler used, but unwind semantics are not enabled
endif()
