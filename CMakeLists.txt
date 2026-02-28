cmake_minimum_required(VERSION 3.18)
project(pd-open303 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64" CACHE STRING "" FORCE)

include(${CMAKE_SOURCE_DIR}/pd.cmake)


add_subdirectory(lib/open303_DSP)

pd_add_external(open303filter~ src/open303filter~.cpp)
target_link_libraries(open303filter_tilde PRIVATE open303_DSP)
