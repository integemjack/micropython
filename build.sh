docker run --rm -it -v .:/project -w /project --privileged integem/pydrone:latest bash -c "cd ports/esp32 && rm -rf build-PYDRONE && make BOARD=PYDRONE"
mkdir -p build
cp -rf ports/esp32/build-PYDRONE/firmware.bin ./build/firmware.bin