#!/bin/bash

if [ $# -eq 0 ]; then
    # echo "No packages specified. Building ALL packages..."
    colcon build \
        --symlink-install \
        --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
else
    colcon build \
        --packages-select "$@" \
        --symlink-install \
        --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi

# ./build.sh camera

# colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# colcon build --packages-select camera image_processor --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON