#/usr/bin/env bash

cur_path="$(dirname "$(readlink -f "$0")")"
cd $cur_path

mkdir build_x64
cd build_x64

cmake .. -DPLATFORM=x64

cmake --build .