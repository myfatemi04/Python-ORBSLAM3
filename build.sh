#!/bin/sh

if [ $(which ninja) ]; then
	NINJA_GENERATOR="-GNinja"
	echo "Using Ninja build generator for faster builds."
fi

cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF $NINJA_GENERATOR
cmake --build build
