### Tutorial to compile `rmgr-ssim-sample.cpp`:
1. compile ssim library first: simple execute `make` in the the root dir (the one have GNUmakefile file).
2. download `stb_image.h` file into `sample/stb_image.h`: `wget https://github.com/nothings/stb/raw/master/stb_image.h -P sample/`
3. then run: `g++ sample/rmgr-ssim-sample.cpp -Iinclude -Llib/linux-amd64-gcc12-make/release/ -l:librmgr-ssim.a -o rmgr-ssim-sample`
