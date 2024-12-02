#/usr/bin/env bash

cur_path="$(dirname "$(readlink -f "$0")")"
cd $cur_path

mkdir build_x86
cd build_x86

cmake .. -DPLATFORM=x86

cmake --build .
