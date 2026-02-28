cmake_minimum_required(VERSION 3.18)
project(open303_DSP)

file(GLOB OPEN_303_DSP_SOURCES ./*.cpp)

add_library(open303_DSP ${OPEN_303_DSP_SOURCES})
target_include_directories(open303_DSP PUBLIC .)
