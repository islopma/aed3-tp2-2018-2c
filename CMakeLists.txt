cmake_minimum_required(VERSION 3.9)
project(tp2_modelado_con_grafos)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_subdirectory(lib/googletest)
include_directories(lib/googletest/googletest/include)
include_directories(lib/googletest/googlemock/include)

set(SOURCE_FILES main.cpp prueba_integracion_con_gtest/test/factorialTest.cpp prueba_integracion_con_gtest/src/factorial.cpp graph.cpp graphTest.cpp )
add_executable(tp2_modelado_con_grafos ${SOURCE_FILES})
target_link_libraries(tp2_modelado_con_grafos gtest gtest_main)