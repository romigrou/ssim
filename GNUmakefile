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

SELF := $(lastword $(MAKEFILE_LIST))
HERE := $(abspath $(dir $(SELF)))

MODE      ?= release
CMAKE     ?= cmake
NINJA     ?= ninja
JFLAG      = $(findstring j,$(firstword $(MAKEFLAGS)))
COLOR     ?= $(if $(JFLAG),0,1)
MAKE_TYPE ?= Unix
V         ?= 0
VERBOSE   ?= $(V)
NO_ASM    ?= 0

RMGR_SSIM_USE_OPENMP ?= ON
RMGR_SSIM_USE_DOUBLE ?= OFF

GTEST_FILTER  ?= *
CMAKE_OPTIONS := "-DRMGR_SSIM_BUILD_TESTS=ON" "-DRMGR_SSIM_USE_OPENMP=$(RMGR_SSIM_USE_OPENMP)" "-DRMGR_SSIM_USE_DOUBLE=$(RMGR_SSIM_USE_DOUBLE)"

SOURCE_DIR    := $(HERE)
CMAKE_TARGETS := rmgr-ssim-tests rmgr-ssim
CMAKELISTS    := $(SOURCE_DIR)/CMakeLists.txt
BUILD_PATH     = build/$(BUILD_DIR)/$(MODE)

CMAKE_ARCHIVE_OUTPUT_DIRECTORY = $(HERE)/lib/$(BUILD_DIR)/$(MODE)
CMAKE_LIBRARY_OUTPUT_DIRECTORY = $(HERE)/bin/$(BUILD_DIR)/$(MODE)
CMAKE_RUNTIME_OUTPUT_DIRECTORY = $(HERE)/bin/$(BUILD_DIR)/$(MODE)
CMAKE_INSTALL_PREFIX           = $(HERE)/package/$(PLATFORM)$(PLATFORM_SUFFIX)-$(ARCH)-$(COMPILER_NAME)
CPACK_PACKAGE_DIRECTORY        = $(HERE)/package
CPACK_SYSTEM_NAME              = $(PLATFORM)$(PLATFORM_SUFFIX)-$(ARCH)-$(COMPILER_NAME)


###############################################################################
# Detect host platform & architecture

ifeq ($(findstring cygdrive,$(PATH)),cygdrive)
    HOST_PLATFORM := cygwin
else ifeq ($(OSTYPE),cygwin)
    HOST_PLATFORM := cygwin
else ifeq ($(OS),Windows_NT)
    HOST_PLATFORM := windows
    HOST_ARCH     := $(PROCESSOR_ARCHITECTURE)
else
    HOST_PLATFORM := $(shell uname -o)
endif
HOST_ARCH ?= $(shell uname -m)


###############################################################################
# Handle Android

ifeq ($(PLATFORM),android)
    ifeq ($(ANDROID_NDK),)
        $(error Set ANDROID_NDK to the location of your NDK)
    else ifeq ($(wildcard $(ANDROID_NDK)/build/cmake/android.toolchain.cmake),)
        $(error ANDROID_NDK seems not to point to a valid NDK)
    endif
    CMAKE_TOOLCHAIN_FILE=$(ANDROID_NDK)/build/cmake/android.toolchain.cmake

    # Select platform level
    ifndef ANDROID_PLATFORM
        ifdef ANDROID_NATIVE_API_LEVEL
            ANDROID_PLATFORM := android-$(lastword $(subst -, ,$(ANDROID_NATIVE_API_LEVEL)))
        else
            # Default to level #21 as this is the first one with support for 64-bit platforms
            ANDROID_PLATFORM := android-21
        endif
    endif
    PLATFORM_SUFFIX := $(lastword $(subst -, ,$(ANDROID_PLATFORM)))

    # Figure out architecture from ABI
    ifneq ($(ANDROID_ABI),)
        ifneq ($(findstring arm64,$(ANDROID_ABI)),)
            ARCH              := aarch64
            TOOLCHAIN_PATTERN := aarch64-linux-android-*
        else ifneq ($(findstring arm,$(ANDROID_ABI)),)
            ARCH              := arm
            TOOLCHAIN_PATTERN := arm-linux-androideabi-*
        else ifneq ($(findstring x86_64,$(ANDROID_ABI)),)
            ARCH              := amd64
            TOOLCHAIN_PATTERN := x86_64-*
        else ifneq ($(findstring x86,$(ANDROID_ABI)),)
            ARCH              := x86
            TOOLCHAIN_PATTERN := x86-*
        else
            $(error Cannot figure out architecture from the value of ANDROID_ABI: $(ANDROID_ABI))
        endif
    endif

    # Look at the toolchain if specified
    ifeq ($(ANDROID_TOOLCHAIN_NAME),)
        COMPILER_NAME := clang
    else
        ifneq ($(findstring $(ANDROID_TOOLCHAIN_NAME),clang))
            COMPILER_NAME := clang
        else
            COMPILER_NAME := gcc$(lastword $(subst -, ,$(ANDROID_TOOLCHAIN_NAME)))
        endif
        ARCH ?= $(firstword $(subst -, ,$(ANDROID_TOOLCHAIN_NAME)))
        ARCH := $(subst x64_64,amd64,$(ARCH))
    endif

    ARCH ?= arm
endif


###############################################################################
# Ensure some consistency between C and C++ compilers (both GCC or both Clang)

DEFAULT_CC  := cc
DEFAULT_CXX := g++
ifeq ($(CC),$(DEFAULT_CC))
    ifneq ($(CXX),$(DEFAULT_CXX))
        # If CXX is g++ or clang++, set CC to gcc or clang, respectively
        ifneq ($(findstring clang++,$(CXX)),)
            export CC := $(subst clang++,clang,$(CXX))
        else ifneq ($(findstring g++,$(CXX)),)
            export CC := $(subst g++,gcc,$(CXX))
        endif
    endif
endif
ifeq ($(CXX),$(DEFAULT_CXX))
    ifneq ($(CC),$(DEFAULT_CC))
        # If CC is gcc or clang, set CXX to g++ or clang++, respectively
        ifneq ($(findstring clang,$(CC)),)
            export CXX := $(subst clang,clang++,$(CC))
        else ifneq ($(findstring gcc,$(CC)),)
            export CXX := $(subst gcc,g++,$(CC))
        endif
    endif
endif


###############################################################################
# Identify compiler

# If host platform is Windows and both compilers are default, then compiler is probably MSVC
ifeq (windows$(DEFAULT_CC)$(DEFAULT_CXX),$(HOST_PLATFORM)$(CC)$(CXX))
    CL_RESULT := $(shell cl.exe 2>&1)
    # Some versions have an extra "32-bit" in the copyright message. Let's remove it.
    CL_RESULT := $(subst 32-bit ,,$(CL_RESULT))
    ifneq ($(findstring Microsoft (R) C/C++,$(CL_RESULT)),)
        # Now, let's try to figure out the version
        ARCH          := $(word 9,$(CL_RESULT))
        CL_VERSION    := $(subst ., ,$(word 7,$(CL_RESULT)))
        CL_MAJOR      := $(firstword $(CL_VERSION))
        CL_MINOR      := $(word 2,$(CL_VERSION))
        COMPILER_NAME := msvc$(CL_MAJOR).$(CL_MINOR)
    endif
endif

# Other compilers
ifndef COMPILER_NAME
    ifneq ($(CC),$(DEFAULT_CC))
        COMPILER_NAME := $(CC)
    else
        COMPILER_NAME := $(CXX)
    endif

    COMPILER_NAME := $(notdir $(basename $(COMPILER_NAME)))
    ifneq ($(findstring mingw32,$(COMPILER_NAME)),)
        # MinGW implies Windows and GCC or Clang
        COMPILER_NAME        := $(filter gcc g++ clang,$(subst -, ,$(COMPILER_NAME)))
        PLATFORM             ?= windows
        CMAKE_TOOLCHAIN_FILE := $(abspath .)/cmake/Toolchain-mingw.cmake
    endif

    # Use canonical compiler name (gcc or clang, not g++ nor clang++)
    COMPILER_NAME := $(subst clang++,clang,$(COMPILER_NAME))
    COMPILER_NAME := $(subst g++,gcc,$(COMPILER_NAME))

    # Figure out compiler version
    ifneq ($(findstring gcc,$(COMPILER_NAME)),)
        COMPILER_NAME := gcc$(shell $(CC) -dumpversion)
    else ifneq ($(findstring clang,$(COMPILER_NAME)),)
        # Can't use -dumpversion for clang as it reports the emulated GCC version
        ifneq ($(HOST_PLATFORM),windows)
            COMPILER_NAME := clang$(word 3,$(subst ", ,$(shell echo "" | $(CC) -E -dM - | grep __clang_version__)))
        endif
    endif
endif


###############################################################################
# Identify target architecture

ifndef ARCH
    # Ask GCC or Clang what their target is
    ifneq ($(or $(findstring gcc,$(CC)),$(findstring clang,$(CC))),)
        ARCH := $(shell $(CC) -dumpmachine)
    else ifneq ($(or $(findstring g++,$(CXX)),$(findstring clang,$(CXX))),)
        ARCH := $(shell $(CXX) -dumpmachine)
    endif

    # Extract architecture from target
    ifdef ARCH
        ARCH := $(firstword $(subst -, ,$(ARCH)))
    endif
endif
ARCH ?= $(HOST_ARCH)

# Normalize architecture names
X86_ARCH_NAMES   := i386 i486 i586 i686 80x86
AMD64_ARCH_NAMES := x86_64 x64 AMD64
ARCH := $(if $(filter $(ARCH),$(X86_ARCH_NAMES)),x86,$(ARCH))
ARCH := $(if $(filter $(ARCH),$(AMD64_ARCH_NAMES)),amd64,$(ARCH))


###############################################################################
# Identify target platform

PLATFORM ?= $(HOST_PLATFORM)
ifeq ($(PLATFORM),GNU/Linux)
    PLATFORM := linux
endif


###############################################################################
# Configure

BUILD_DIR := $(PLATFORM)$(PLATFORM_SUFFIX)-$(ARCH)-$(COMPILER_NAME)

# Set up builder (make or ninja)
ifeq ($(MAKE_TYPE),Ninja)
    GENERATOR           := Ninja
    BUILDER             := $(NINJA)
    BUILDER_OPTIONS      = $(NINJA_VERBOSE) $(NINJA_JFLAG)
    BUILD_DIR           := $(BUILD_DIR)-ninja
    NINJA_JFLAG          = $(if $(JFLAG),,-j 1)
else
    GENERATOR           := $(MAKE_TYPE) Makefiles
    BUILDER             := $(MAKE)
    BUILDER_OPTIONS      = $(MAKE_VERBOSE)
    BUILD_DIR           := $(BUILD_DIR)-make
    CMAKE_COLOR_MAKEFILE = -DCMAKE_COLOR_MAKEFILE=$(COLOR)
endif

ifneq ($(NO_ASM),0)
    BUILD_DIR := $(BUILD_DIR)-noasm
endif

CMAKE_OPTIONS += -DCMAKE_BUILD_TYPE=$(MODE)
CMAKE_OPTIONS += -DCMAKE_BUILD_TOOL=$(MAKE)
CMAKE_OPTIONS += $(if $(CMAKE_TOOLCHAIN_FILE),"-DCMAKE_TOOLCHAIN_FILE:FILEPATH=$(CMAKE_TOOLCHAIN_FILE)",)
CMAKE_OPTIONS += "-DCMAKE_ARCHIVE_OUTPUT_DIRECTORY:PATH=$(CMAKE_ARCHIVE_OUTPUT_DIRECTORY)"
CMAKE_OPTIONS += "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY:PATH=$(CMAKE_LIBRARY_OUTPUT_DIRECTORY)"
CMAKE_OPTIONS += "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH=$(CMAKE_RUNTIME_OUTPUT_DIRECTORY)"
CMAKE_OPTIONS += $(CMAKE_COLOR_MAKEFILE)

CMAKE_ANDROID_OPTIONS :=
ifeq ($(PLATFORM),android)
    ANDROID_VARS := NDK ABI ALLOW_UNDEFINED_SYMBOLS APP_PIE ARM_MODE ARM_NEON CCACHE CPP_FEATURES DISABLE_FORMAT_STRING_CHECKS NATIVE_API_LEVEL PLATFORM STL TOOLCHAIN_NAME
    CMAKE_ANDROID_OPTIONS := $(foreach var,$(ANDROID_VARS),$(if $(ANDROID_$(var)),"-DANDROID_$(var)=$(ANDROID_$(var))",))
endif

# Verbosity level
ifeq ($(VERBOSE),0)
    MAKE_VERBOSE  := -s VERBOSE=0
    NINJA_VERBOSE :=
.SILENT:
else
    MAKE_VERBOSE  := VERBOSE=1
    NINJA_VERBOSE := -v
endif

###############################################################################
# Rules

# Macro for forwarding a target to the CMake-generated makefile
define cmake_target
.PHONY: $(1)
$(1): $(BUILD_PATH)/CMakeCache.txt
	+$(BUILDER) -C $(BUILD_PATH) $(BUILDER_OPTIONS) $(1)

.PHONY: rebuild-$(1)
rebuild-$(1): clobber
	+$(MAKE) -f $(SELF) $(MAKE_VERBOSE) $(1)

endef

# Apply the above macro to CMake targets
#$(foreach target,$(CMAKE_TARGETS),$(info $(call cmake_target,$(target))))
$(foreach target,$(CMAKE_TARGETS),$(eval $(call cmake_target,$(target))))

# Hand-defined rules
.PHONY: run-tests
run-tests: $(firstword $(CMAKE_TARGETS))
	$(subst build,bin,$(BUILD_PATH))/$(firstword $(CMAKE_TARGETS)) --gtest_filter=$(GTEST_FILTER)

.PHONY: clean
clean: $(BUILD_PATH)/CMakeCache.txt
	+$(BUILDER) -C $(BUILD_PATH) $(BUILDER_OPTIONS) clean

.PHONY: clobber
clobber:
	-+$(BUILDER) -C $(BUILD_PATH) $(BUILDER_OPTIONS) clean
	-$(CMAKE) -E remove_directory $(BUILD_PATH)

.PHONY: rebuild
rebuild: clobber
	+$(MAKE) -f $(SELF) $(MAKE_VERBOSE)

.PHONY: build-files
build-files: $(BUILD_PATH)/CMakeCache.txt

$(BUILD_PATH)/CMakeCache.txt:
	-$(CMAKE) -E make_directory $(BUILD_PATH)
	cd $(BUILD_PATH) && $(CMAKE) -G "$(GENERATOR)" $(CMAKE_OPTIONS) $(CMAKE_ANDROID_OPTIONS) $(SOURCE_DIR)

# Writes to the console the makefile's internal variables
.PHONY: showvars
showvars:
	$(info MAKE                           = $(MAKE))
	$(info MAKEFLAGS                      = $(MAKEFLAGS))
	$(info JFLAG                          = $(JFLAG))
	$(info MODE                           = $(MODE))
	$(info CMAKE                          = $(CMAKE))
	$(info NINJA                          = $(NINJA))
	$(info COLOR                          = $(COLOR))
	$(info MAKE_TYPE                      = $(MAKE_TYPE))
	$(info V                              = $(V))
	$(info VERBOSE                        = $(VERBOSE))
	$(info CC                             = $(CC))
	$(info CXX                            = $(CXX))
	$(info COMPILER_NAME                  = $(COMPILER_NAME))
	$(info HOST_PLATFORM                  = $(HOST_PLATFORM))
	$(info HOST_ARCH                      = $(HOST_ARCH))
	$(info ARCH                           = $(ARCH))
	$(info PLATFORM                       = $(PLATFORM))
	$(info BUILD_DIR                      = $(BUILD_DIR))
	$(info GENERATOR                      = $(GENERATOR))
	$(info BUILDER                        = $(BUILDER))
	$(info BUILDER_OPTIONS                = $(BUILDER_OPTIONS))
	$(info CMAKE_ARCHIVE_OUTPUT_DIRECTORY = $(CMAKE_ARCHIVE_OUTPUT_DIRECTORY))
	$(info CMAKE_LIBRARY_OUTPUT_DIRECTORY = $(CMAKE_LIBRARY_OUTPUT_DIRECTORY))
	$(info CMAKE_RUNTIME_OUTPUT_DIRECTORY = $(CMAKE_RUNTIME_OUTPUT_DIRECTORY))
	$(info CMAKE_INSTALL_PREFIX           = $(CMAKE_INSTALL_PREFIX))
	$(info CPACK_PACKAGE_DIRECTORY        = $(CPACK_PACKAGE_DIRECTORY))
	$(info CPACK_SYSTEM_NAME              = $(CPACK_SYSTEM_NAME))
	$(info CMAKE_ANDROID_OPTIONS          = $(CMAKE_ANDROID_OPTIONS))
