mkdir -p build
cd build
echo "\nBuilding executable...\n"
cmake ..
make
cd ..
echo "\nRunning executable...\n"
./build/ntscpp
