cmake_minimum_required(VERSION 3.9)
project(tp2_modelado_con_grafos)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall")

add_subdirectory(lib/googletest)
include_directories(lib/googletest/googletest/include)
include_directories(lib/googletest/googlemock/include)

set(SOURCE_FILES main.cpp prueba_integracion_con_gtest/test/factorialTest.cpp prueba_integracion_con_gtest/src/factorial.cpp utils/src/graph.cpp utils/test/graphTest.cpp utils/test/MSTTest.cpp utils/src/DisjoinSet.cpp utils/test/testBellman.cpp utils/test/DisjoinSetTest.cpp)
add_executable(tp2_modelado_con_grafos ${SOURCE_FILES})
add_executable(clustering utils/src/graph.cpp utils/src/DisjoinSet.cpp utils/src/clustering/clustering.cpp utils/src/clustering/point.cpp utils/src/clustering/runner.cpp)
add_executable(arbitraje utils/src/graph.cpp utils/src/DisjoinSet.cpp utils/src/arbitraje/runner.cpp)
add_executable(arbitraje_time utils/src/graph.cpp utils/src/DisjoinSet.cpp utils/src/arbitraje/time.cpp)
target_link_libraries(tp2_modelado_con_grafos gtest gtest_main)