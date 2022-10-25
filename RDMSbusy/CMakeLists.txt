cmake_minimum_required(VERSION 3.20)
project(ETA)

set(BOOST_ROOT /usr/local/Cellar/boost/1.76.0)
set(Boost_INCLUDE_DIR /usr/local/Cellar/boost/1.76.0/include)
set(Boost_LIBRARY_DIR /usr/local/Cellar/boost/1.76.0/lib)
find_package(Boost 1.76.0  COMPONENTS thread system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

set(CMAKE_CXX_STANDARD 14)

link_directories(/usr/local/Cellar/boost/1.76.0/lib/)
add_executable(ETA main.cpp head.h Alg2.cpp Alg1.cpp structure.cpp od_generation.cpp)

target_link_libraries(ETA boost_thread-mt boost_system)
