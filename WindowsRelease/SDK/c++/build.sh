#bin/bash

rm -r ./../../replay/*
# rm -rf ./build/*
cd build
cmake .. -G "MinGW Makefiles"
mingw32-make -j