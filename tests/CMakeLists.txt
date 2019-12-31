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
        set(GTEST_VERSION 1.8.1)
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
# stb_image

set(STBI_HASH "f67165c2bb2af3060ecae7d20d6f731173485ad0")
set(STBI_URL  "https://raw.githubusercontent.com/nothings/stb/${STBI_HASH}/stb_image.h")
set(STBI_PATH "${CMAKE_CURRENT_SOURCE_DIR}/stb_image.h")
if (NOT EXISTS "${STBI_PATH}")
    file(DOWNLOAD "${STBI_URL}" "${STBI_PATH}" STATUS STBI_STATUS)
    list(GET STBI_STATUS 0 STBI_STATUS_VALUE)
    if (NOT STBI_STATUS_VALUE EQUAL 0)
        list(GET STBI_STATUS 1 STBI_STATUS_MSG)
        message(FATAL_ERROR "Failed to download stb_image.h: ${STBI_STATUS_MSG}")
        if (EXISTS "${STBI_PATH}")
            file(REMOVE "${STBI_PATH}")
        endif()
    endif()
endif()


###############################################################################
# Main Target

get_filename_component(RMGR_SSIM_TESTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)

set(RMGR_SSIM_TESTS_FILES
    "rmgr-ssim-tests.cpp"
    "stb_image.h"
)

source_group("Source Files" FILES ${RMGR_SSIM_TESTS_FILES})

add_executable(rmgr-ssim-tests ${RMGR_SSIM_TESTS_FILES})

target_link_libraries(rmgr-ssim-tests PRIVATE
    ${GTEST_LIBRARIES}
    rmgr-ssim
)

target_include_directories(rmgr-ssim-tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_compile_options(rmgr-ssim-tests PRIVATE ${RMGR_SSIM_COMPILE_OPTIONS})
target_compile_definitions(rmgr-ssim-tests PRIVATE "RMGR_SSIM_TESTS_DIR=\"${RMGR_SSIM_TESTS_DIR}\"" ${RMGR_SSIM_DEFINITIONS})

if (MSVC)
    # Silence warnings
    target_compile_options(rmgr-ssim-tests PRIVATE "/wd4530") # C++ exception handler used, but unwind semantics are not enabled
endif()
