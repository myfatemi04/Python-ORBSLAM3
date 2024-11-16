#!/bin/sh

if [ $(which ninja) ]; then
	NINJA_GENERATOR="-GNinja"
	echo "Using Ninja build generator for faster builds."
fi

PYTHON_ROOT_DIR=$(which python)/../..

cmake -B build -S . \
	-DCMAKE_BUILD_TYPE=Release \
	-DBUILD_TESTS=OFF \
	-DBUILD_EXAMPLES=OFF \
	-DCMAKE_INSTALL_PREFIX=install \
	-DPython_ROOT_DIR=$PYTHON_ROOT_DIR \
	$NINJA_GENERATOR

cmake --build build
cmake --install build
