# Specify the minimum version of CMake required
cmake_minimum_required(VERSION 3.20)

# Include the Zephyr build system
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Specify the project name
project(lp5810-zephyr-driver)
#add_subdirectory(drivers) # for some reason this does not work instead I have added drivers/led/lp5810.c to target_sources

# Add your source files here
target_sources(app PRIVATE src/main.c drivers/led/lp5810.c)

