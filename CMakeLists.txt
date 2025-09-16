#================================================================================
cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE cmake/arm-none-eabi-gcc.cmake)


#================================================================================
project(es LANGUAGES CXX C ASM)
set(DEVICE stm32f407vg)


#================================================================================

add_subdirectory(src)

add_subdirectory(libs)

add_subdirectory(flash_n_debug)

add_subdirectory(doc)
