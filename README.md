rmgr::ssim
==========

This is a small and fast library whose only purpose in life is to compute the [SSIM](https://ece.uwaterloo.ca/~z70wang/research/ssim)
(Structural Similarity) of two images. SSIM is intended more to closely model human perception than
[PSNR](https://en.wikipedia.org/wiki/Peak_signal-to-noise_ratio) (Peak Signal to Noise Ratio) does.


Portability
-----------
The library is written in C++ 98 so as to make it as portable as possible and has no dependency beyond
compiler-provided headers and libraries. It is tested against several different compilers (namely Visual C++,
GCC and Clang) and with several versions of each on several different platforms.


Usage
-----
For the common use case, there's only one function you will need: `rmgr::ssim::compute_ssim()`. Granted,
it takes quite a few parameters but that's the price to pay for flexibility and there's nothing really
complicated about them.

`compute_ssim()` can optionally compute the per-pixel SSIM map (see below), which has a small impact on
performance, but can come in handy.

The default behaviour is to make no allocation and only use stack-based storage, which is fine if you
have at least 400 KiB of stack space. If that's a lot for your stack to handle, you can have the function
allocate buffers off the heap instead.


Speed
-----
SSIM is not exactly renowned for its speed, each pixel requiring 1230 arithmetic operations (which can be
lowered by exploiting symmetries, but this is to be compared with PSNR requiring only 2 operations). With this
in mind, the library has been thoroughly optimized in order to make SSIM computation as light as possible.

The library can use the SSE, AVX or FMA instruction sets (AVX-512 is not yet supported) with runtime
detection of which is available. Multi-threaded operation is also possible using OpenMP
(see below how to enable support).

### Figures
Here is a table of performance figures on my laptop's [Core i7-4710HQ](https://ark.intel.com/content/www/us/en/ark/products/78930/intel-core-i7-4710hq-processor-6m-cache-up-to-3-50-ghz.html):

|        | Without map |  With map   | OpenMP w/o map | OpenMP w/ map |
|--------|------------:|------------:|---------------:|--------------:|
|Generic |  9.3 Mpix/s |  9.1 Mpix/s |   40.6 Mpix/s  |   39.8 Mpix/s |
|SSE     | 34.7 Mpix/s | 34.2 Mpix/s |  144.2 Mpix/s  |  135.1 Mpix/s |
|AVX     | 53.3 Mpix/s | 51.9 Mpix/s |  223.5 Mpix/s  |  206.0 Mpix/s |
|FMA     | 57.3 Mpix/s | 55.6 Mpix/s |  232.4 Mpix/s  |  223.4 Mpix/s |

An ARM version is also available with Neon (or ASIMD) support to speed up operation.
The following are performance figures from a quad-core Cortex A53 @ 1.8 GHz, in both 32 and 64-bit modes

|                | Without map |  With map   | OpenMP w/o map | OpenMP w/ map |
|----------------|------------:|------------:|---------------:|--------------:|
| 32-bit Generic |  1.4 Mpix/s |  1.4 Mpix/s |     5.6 Mpix/s |    5.5 Mpix/s |
| 64-bit Generic |  1.3 Mpix/s |  1.3 Mpix/s |     9.0 Mpix/s |    8.8 Mpix/s |
| 32-bit Neon    |  3.1 Mpix/s |  1.3 Mpix/s |    11.5 Mpix/s |   11.3 Mpix/s |
| 64-bit ASIMD   |  6.5 Mpix/s |  6.4 Mpix/s |    30.7 Mpix/s |   29.9 Mpix/s |

### Comparison
How does this compare to other versions available out there? I have only tested two of them:
  - An [OpenCV](http://opencv.org/)-based one, written by [Mehdi Rabah](http://mehdi.rabah.free.fr/SSIM/), that's listed on
    SSIM's official page. It is based on the reference [MATLAB code from Zhou Wang](https://www.cns.nyu.edu/~lcv/ssim/ssim_index.m).
    It runs at around **22 Mpix/s**, which is quite good, especially considering how small is the code.
    Note that due to the code being quite old I could only test it with Open CV 3.4.8, more recent versions
    might fare better.
  - The one from [Basis](https://github.com/BinomialLLC/basis_universal). This one is a full C++
    re-implementation that has no outside dependency. It does not use SIMD nor multi-threading
    and runs at around **1.5 Mpix/s**.

There are several other SSIM implementations you can find on GitHub. Most of them are similar to Mehdi Rabah's
and rely on OpenCV, they should therefore have comparable performance figures.


Accuracy
--------
The API only deals with `float`, because 6 digits is enough precision for SSIM values. However, internal
computations are done with more precision. The default is to perform all per-pixel intermediate computations
in single precision and only the sum for the global SSIM is done in double precision.

Using the `RMGR_SSIM_USE_DOUBLE` option, you can force all computations to be carried out in double precision.
But be aware that this will basically make the library run half as fast.

The following table gives the maximum errors of global and per-pixel SSIMs depending on whether the library
was built with single or double internal precision. Those errors are relative to a fully quad-precision (128-bit)
reference implementation. As you can see, the double precision is almost as precise as the `float` API allows;
The single precision version should nonetheless be precise enough for most applications.

|                  | Average<br>Global Error | Maximum<br>Global Error | Average<br>Per-Pixel Error | Maximum<br>Per-Pixel Error |
|:-----------------|:-----------------------:|:-----------------------:|:--------------------------:|:--------------------------:|
| Single Precision |               1.75×10⁻⁷ |               1.49×10⁻⁶ |                  5.61×10⁻⁶ |                  6.22×10⁻⁴ |
| Double Precision |               1.31×10⁻⁷ |               4.75×10⁻⁷ |                  7.35×10⁻⁸ |                  9.21×10⁻⁶ |


Multi-Channel Images
--------------------
SSIM is only defined for grayscale images and this is all that the library supports. A common way of handling
color images is to convert them to luminance. You may also compute the SSIM of each channel and somehow
merge them into a composite SSIM (taking the smallest value for example, or making a weighted sum).
You can even take this a step further using the SSIM map: compute per-pixel composite SSIMs and
then compute a global composite SSIM (the global SSIM is simply the average of per-pixel SSIMs).

For alpha channel, just as with color, you're on your own. Try to figure out some way of computing a
composite SSIM from per-channel SSIMs.


Bit Depth Support
-----------------
Only 8-bit images are supported. However, it shouldn't be complicated to modify the code to support
other bit depths: all that needs to be changed are the <code>[L](https://github.com/romigrou/ssim/blob/8d684ba783a74540a8b3be748e9fcf8368be509f/src/ssim.cpp#L894)</code>
constant and the <code>[retrieve_tile()](https://github.com/romigrou/ssim/blob/8d684ba783a74540a8b3be748e9fcf8368be509f/src/ssim.cpp#L451)</code> function.

Building
--------

The library relies on [CMake](https://cmake.org/) to build. SIMD intrinsics will be used whenever possible
and [OpenMP](https://www.openmp.org/) can be enabled by turning on a simple option (`RMGR_SIMD_USE_OPENMP`).

As mentioned above, for improved precision, you can have the library perform all the computations using `double`
rather than `float` by setting the `RMGR_SSIM_USE_DOUBLE` option to `ON`.

Otherwise, the code is mostly self-configurable and should therefore be easy to port to another build system.

Command Line Tool
-----------------

A simple command line tool that computes the SSIM of two images is available. To build it, set the CMake variable
`RMGR_SSIM_BUILD_CLI` to `ON`.

Sample Code
-----------

<code>[sample/rmgr-ssim-sample.cpp](sample/rmgr-ssim-sample.cpp)</code> is a simplified version of the aforementioned
command line tool that showcases how to call `rmgr::ssim::compute_ssim()` in the most straightforward manner.
For sample code of more advanced uses, please refer to [the unit tests](tests/rmgr-ssim-tests.cpp).

Documentation
-------------

You can generate the documentation by running doxygen in the root folder, the HTML documentation
will end up in the `doc` folder.


Licensing
---------

This library is licensed under the zlib license, as follows:

> Copyright © 2021, Romain Bailly
>
> This software is provided 'as-is', without any express or implied
> warranty.  In no event will the authors be held liable for any damages
> arising from the use of this software.
>
> Permission is granted to anyone to use this software for any purpose,
> including commercial applications, and to alter it and redistribute it
> freely, subject to the following restrictions:
>
> 1. The origin of this software must not be misrepresented; you must not
>    claim that you wrote the original software. If you use this software
>    in a product, an acknowledgment in the product documentation would be
>    appreciated but is not required.
> 2. Altered source versions must be plainly marked as such, and must not be
>    misrepresented as being the original software.
> 3. This notice may not be removed or altered from any source distribution.
