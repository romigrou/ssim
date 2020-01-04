:: Copyright (c) 2020, Romain Bailly
::
:: This software is provided 'as-is', without any express or implied
:: warranty. In no event will the authors be held liable for any damages
:: arising from the use of this software.
::
:: Permission is granted to anyone to use this software for any purpose,
:: including commercial applications, and to alter it and redistribute it
:: freely, subject to the following restrictions:
::
:: 1. The origin of this software must not be misrepresented; you must not
::    claim that you wrote the original software. If you use this software
::    in a product, an acknowledgment in the product documentation would be
::    appreciated but is not required.
:: 2. Altered source versions must be plainly marked as such, and must not be
::    misrepresented as being the original software.
:: 3. This notice may not be removed or altered from any source distribution.

@echo off

setlocal EnableDelayedExpansion

:: Set default values
if "%CMAKE%"=="" set CMAKE=cmake
set GENERATOR=
set SOURCE_DIR=%~dp0
set BUILD_DIR=%~dp0\build\
set BIN_DIR=%~dp0bin
set LIB_DIR=%~dp0lib
set PKG_DIR=%~dp0package
set CMAKE_OPTIONS= -DRMGR_SSIM_BUILD_TESTS=ON -DRMGR_SSIM_USE_OPENMP=ON


:: List of VS versions, can be any combination of 2003 2005 2008 2010 2012 2013 2015 2017 2019
set DESIRED_YEARS=2005 2008 2010 2012 2013 2015 2017 2019

:: List of VS platforms, can be any combination of Win32 x64 IA64 ARM ARM64
set DESIRED_PLATFORMS=Win32 x64 ARM ARM64

:: Normalize paths
for %%v in (SOURCE_DIR BUILD_DIR BIN_DIR LIB_DIR PKG_DIR) do (
	if "!%%v:~-1!"=="\" set %%v=!%%v:~0,-1!
	for %%p in ("!%%v!") do (
		set %%v=%%~fp
	)
)


:: ============================================================================
:: Parse command line options

set VS_LABEL=
set PLATFORM_LABEL=
set AUTOMATED=0
set CMAKE_USER_OPTIONS=
:ParseOptions
if not "%~1"=="" (
	if "%~1"=="--" (
		shift
		goto ParseCMakeOptions
	)

	if "%VS_LABEL%"=="" (
		set VS_LABEL=%~1
	) else if "%PLATFORM_LABEL%"=="" (
		set PLATFORM_LABEL=%~1
		set AUTOMATED=1
	) else (
		echo Too many arguments on the command line 1>&2
		goto Error
	)
	shift
	goto ParseOptions
)

:ParseCMakeOptions
if not "%~1"=="" (
	set CMAKE_USER_OPTIONS=%CMAKE_USER_OPTIONS% %1
	shift
	goto ParseCMakeOptions
)


:: ============================================================================
:: Choose Visual Studio version

if not "%VS_LABEL%"=="" goto %VS_LABEL%

set VS2003_INDEX=0
set VS2005_INDEX=0
set VS2008_INDEX=0
set VS2010_INDEX=0
set VS2012_INDEX=0
set VS2013_INDEX=0
set VS2015_INDEX=0
set VS2017_INDEX=0
set VS2019_INDEX=0

set INDEX=1
for %%y in (%DESIRED_YEARS%) do (
	set VS%%y_INDEX=!INDEX!
	set /a INDEX=!INDEX!+1
	set VS_YEAR=%%y
)
if %INDEX% equ 1 goto VS%VS_YEAR%

:ChooseGenerator
echo Which version of Visual Studio do you want to generate a solution for?
if %VS2003_INDEX% NEQ 0 echo  %VS2003_INDEX%- Visual Studio 7.1 (2003)
if %VS2005_INDEX% NEQ 0 echo  %VS2005_INDEX%- Visual Studio 8   (2005)
if %VS2008_INDEX% NEQ 0 echo  %VS2008_INDEX%- Visual Studio 9   (2008)
if %VS2010_INDEX% NEQ 0 echo  %VS2010_INDEX%- Visual Studio 10  (2010)
if %VS2012_INDEX% NEQ 0 echo  %VS2012_INDEX%- Visual Studio 11  (2012)
if %VS2013_INDEX% NEQ 0 echo  %VS2013_INDEX%- Visual Studio 12  (2013)
if %VS2015_INDEX% NEQ 0 echo  %VS2015_INDEX%- Visual Studio 14  (2015)
if %VS2017_INDEX% NEQ 0 echo  %VS2017_INDEX%- Visual Studio 15  (2017)
if %VS2019_INDEX% NEQ 0 echo  %VS2019_INDEX%- Visual Studio 16  (2019)
set CHOICE=
set /p CHOICE=
echo.
if "%CHOICE%"=="0" goto ChooseGenerator
for %%y in (2003 2005 2008 2010 2012 2013 2015 2017 2019) do (
	if "%CHOICE%"=="!VS%%y_INDEX!" goto VS%%y
)
goto ChooseGenerator

:VS2003
set "GENERATOR=Visual Studio 7 .NET 2003"
set VS_YEAR=2003
set VC_VERSION=13
set SUPPORTED_PLATFORMS=Win32
goto CheckGenerator

:VS2005
set "GENERATOR=Visual Studio 8 2005"
set VS_YEAR=2005
set VC_VERSION=14
set SUPPORTED_PLATFORMS=Win32 x64
goto CheckGenerator

:VS2008
set "GENERATOR=Visual Studio 9 2008"
set VS_YEAR=2008
set VC_VERSION=15
set SUPPORTED_PLATFORMS=Win32 x64 IA64
goto CheckGenerator

:VS2010
set "GENERATOR=Visual Studio 10 2010"
set VS_YEAR=2010
set VC_VERSION=16
set SUPPORTED_PLATFORMS=Win32 x64 IA64
goto CheckGenerator

:VS2012
set "GENERATOR=Visual Studio 11 2012"
set VS_YEAR=2012
set VC_VERSION=17
set SUPPORTED_PLATFORMS=Win32 x64 ARM
goto CheckGenerator

:VS2013
set "GENERATOR=Visual Studio 12 2013"
set VS_YEAR=2013
set VC_VERSION=18
set SUPPORTED_PLATFORMS=Win32 x64 ARM
goto CheckGenerator

:VS2015
set "GENERATOR=Visual Studio 14 2015"
set VS_YEAR=2015
set VC_VERSION=19.0
set SUPPORTED_PLATFORMS=Win32 x64 ARM
goto CheckGenerator

:VS2017
set "GENERATOR=Visual Studio 15 2017"
set VS_YEAR=2017
set VC_VERSION=19.1
set SUPPORTED_PLATFORMS=Win32 x64 ARM ARM64
goto CheckGenerator

:VS2019
set "GENERATOR=Visual Studio 16 2019"
set VS_YEAR=2019
set VC_VERSION=19.2
set SUPPORTED_PLATFORMS=Win32 x64 ARM ARM64
goto CheckGenerator

:CheckGenerator
for %%y in (%DESIRED_YEARS%) do (
	if "%VS_YEAR%"=="%%y" goto ValidGenerator
)
echo VS%VS_YEAR% is not in the list of supported VS versions 1>&2
goto Error
:ValidGenerator


:: ============================================================================
:: Choose platform

if not "%PLATFORM_LABEL%"=="" goto %PLATFORM_LABEL%

set WIN32_INDEX=0
set X64_INDEX=0
set IA64_INDEX=0
set ARM_INDEX=0
set ARM64_INDEX=0
set INDEX=1
for %%p in (%SUPPORTED_PLATFORMS%) do (
	for %%d in (%DESIRED_PLATFORMS%) do (
		if "%%p"=="%%d" (
			set %%p_INDEX=!INDEX!
			set /a INDEX=!INDEX!+1
			set PLATFORM=%%p
		)
	)
)
if %INDEX% equ 0 (
	echo The VS%VS_YEAR% does not support any of the desired platforms 1>&2
	exit /b 1
)
if %INDEX% equ 1 goto %PLATFORM%

:ChoosePlatform
echo Which platform to you want to generate for?
if %WIN32_INDEX% neq 0 echo  %WIN32_INDEX%- x86
if %X64_INDEX%   neq 0 echo  %X64_INDEX%- AMD64
if %IA64_INDEX%  neq 0 echo  %IA64_INDEX%- IA64
if %ARM_INDEX%   neq 0 echo  %ARM_INDEX%- ARM
if %ARM64_INDEX% neq 0 echo  %ARM64_INDEX%- ARM64
set CHOICE=
set /p CHOICE=
echo.
for %%p in (Win32 x64 IA64 ARM ARM64) do (
	if "%CHOICE%"=="!%%p_INDEX!" goto %%p
)
goto ChoosePlatform

:Win32
:x86
set PLATFORM=Win32
set ARCH=x86
goto CheckPlatform

:x64
:amd64
:x86_64
set PLATFORM=x64
set ARCH=amd64
goto CheckPlatform

:ia64
set PLATFORM=IA64
set ARCH=ia64
goto CheckPlatform

:arm
set PLATFORM=ARM
set ARCH=arm
goto CheckPlatform

:arm64
:aarch64
set PLATFORM=ARM64
set ARCH=aarch64
goto CheckPlatform

:CheckPlatform
for %%p in (%SUPPORTED_PLATFORMS%) do (
	for %%d in (%DESIRED_PLATFORMS%) do (
		if "%%p"=="%%d" (
			if "%%p"=="%PLATFORM%" goto ValidPlatform
		)
	)
)
echo %PLATFORM% is not in the list of supported platforms 1>&2
goto Error
:ValidPlatform


:: ============================================================================
:: Generate the solution

if %VS_YEAR% lss 2017 (
	if "%PLATFORM%"=="x64" (
		set GENERATOR=%GENERATOR% Win64
	) else if not "%PLATFORM%"=="Win32" (
		set GENERATOR=%GENERATOR% %PLATFORM%
	)
) else (
	set CMAKE_OPTIONS=%CMAKE_OPTIONS% -A %PLATFORM%
)

set TRIPLET=windows-%ARCH%-msvc%VC_VERSION%
set QUADRUPLET=%TRIPLET%-vs%VS_YEAR%
set BUILD_DIR=%BUILD_DIR%\%QUADRUPLET%
set CMAKE_OPTIONS=%CMAKE_OPTIONS% "-DCMAKE_ARCHIVE_OUTPUT_DIRECTORY:PATH=%LIB_DIR%\%QUADRUPLET%"
set CMAKE_OPTIONS=%CMAKE_OPTIONS% "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY:PATH=%BIN_DIR%\%QUADRUPLET%"
set CMAKE_OPTIONS=%CMAKE_OPTIONS% "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY:PATH=%BIN_DIR%\%QUADRUPLET%"
set CMAKE_OPTIONS=%CMAKE_OPTIONS% "-DCMAKE_INSTALL_PREFIX:PATH=%PKG_DIR%\%TRIPLET%"
set CMAKE_OPTIONS=%CMAKE_OPTIONS% "-DCPACK_PACKAGE_DIRECTORY:PATH=%PKG_DIR%"
set CMAKE_OPTIONS=%CMAKE_OPTIONS% "-DCPACK_SYSTEM_NAME:STRING=%TRIPLET%"
set CMAKE_INVOCATION="%CMAKE%" -G "%GENERATOR%"%CMAKE_OPTIONS%%CMAKE_USER_OPTIONS% "%SOURCE_DIR%"

if exist "%BUILD_DIR%" rmdir /s /q "%BUILD_DIR%"
if errorlevel 1 (echo Failed to delete previous folder "%BUILD_DIR%" && goto Error)

mkdir "%BUILD_DIR%"
if errorlevel 1 (echo Failed to create folder "%BUILD_DIR%" && goto Error)

pushd "%BUILD_DIR%"
if errorlevel 1 (echo Failed to cd to folder "%BUILD_DIR%" && goto Error)

echo Calling CMake as follows:
echo %CMAKE_INVOCATION%
echo.
%CMAKE_INVOCATION%
if errorlevel 1 (popd && goto Error)
popd

if %AUTOMATED% neq 0 exit /b 0
for %%s in ("%BUILD_DIR%\*.sln") do (
	set SLN_PATH=%%s
	goto AskOpen
)
:AskOpen
echo.
echo Do you want to open the solution? (y/n)
set /p CHOICE=
if "%CHOICE%"=="y" (
	start %SLN_PATH%
	exit /b 0
)
if "%CHOICE%"=="n" exit /b 0
goto AskOpen


:Error
if %AUTOMATED% equ 0 pause
exit /b 1
