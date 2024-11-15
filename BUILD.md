# NEW

Run two commands in the root of this directory.
```
cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
cmake --build build
```

# OLD

## DBoW2

Run the following in `Thirdparty/DBoW2`:
```
cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

If you do not have Ninja installed, simply remove the `-GNinja` flag.

## g2o

Run the same commands in `Thirdparty/g2o`.
```
cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Sophus

Run the same commands in `Thirdparty/g2o`.

## Pangolin

Run the following commands in `Thirdparty/Pangolin`:
```
cmake -B build -DCMAKE_INSTALL_PREFIX=$(pwd)/install -DCMAKE_BUILD_TYPE=Release -GNinja
cmake --build build
cmake --install build
```

This will create `bin`, `include`, and `lib` folders inside `Thirdparty/Pangolin/install`.
