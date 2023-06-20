cmake_minimum_required(VERSION 3.13)
project(thermalcam)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
pkg_check_modules(SDL2_ttf REQUIRED IMPORTED_TARGET SDL2_ttf)
find_package(OpenSSL REQUIRED)



include_directories(
        /usr/include
        ${SDL2_INCLUDE_DIRS}
        ${SDL2_ttf_INCLUDE_DIRS}
        3rdparty/mlx90640/include

)

link_directories(
        /usr/lib
        3rdparty/mlx90640/lib
)

# ============================================================================
# ------------------------------ Build camera driver and API -----------------

add_library(mlx90640_api STATIC
        3rdparty/mlx90640/src/MLX90640_API.cpp
        3rdparty/mlx90640/src/MLX90640_LINUX_I2C_Driver.cpp
        3rdparty/mlx90640/include/MLX90640_API.h
        3rdparty/mlx90640/include/MLX90640_I2C_Driver.h)

# ============================================================================
# ------------------------------ Build application ---------------------------

add_executable(ThermalCamera
        src/ThermalCamera.cpp
        src/main.cpp
        src/constants.h
        src/colormap.h

)

# Set the output directory for the executable

#set_target_properties(ThermalCamera PROPERTIES
#    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../"
#)

target_link_libraries(ThermalCamera
        mlx90640_api
        PkgConfig::SDL2
        PkgConfig::SDL2_ttf
        pigpio
        pthread
        /usr/local/lib/arm-linux-gnueabihf/libTgBot.a
        ${CMAKE_THREAD_LIBS_INIT}
        ${OPENSSL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${CURL_LIBRARIES}
)

