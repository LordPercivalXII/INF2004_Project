rm -rf build
mkdir build
cmake -S . -B build -G Ninja -DROBOT_SIMULATION=ON -DPICO_BOARD=pico_w
cd build
ninja -j8