#!/usr/bin/env bash

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4