project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/planner.cpp src/planner.h)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


#if (BUILD_TESTS)
    add_library(Catch INTERFACE)
    target_include_directories(Catch INTERFACE tests)

    add_executable(path_planner_test ${sources} tests/planner_tests.cpp)
    target_link_libraries(path_planner_test Catch)

    enable_testing()
    add_test(NAME planner_tests COMMAND path_planner_test)

#endif(BUILD_TESTS)


add_executable(path_planning src/main.cpp ${sources})

target_link_libraries(path_planning z ssl uv uWS)
