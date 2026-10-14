rm -rf build
mkdir build
cmake -S . -B build -G "Unix Makefiles" -DROBOT_SIMULATION=ON  -DPICO_BOARD=pico_w
cd build
make -j8