cmake_minimum_required(VERSION 2.8.6)
project(fastmassspring)

add_compile_options(-fPIC)
add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)

add_definitions(-DOCTOCON_DIR="${CMAKE_HOME_DIRECTORY}")
set(CMAKE_MODULE_PATH "${CMAKE_HOME_DIRECTORY}/cmake_module")

add_subdirectory( sim )
add_subdirectory( render )