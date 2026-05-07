cmake_minimum_required(VERSION 3.22...3.31)

set(NAME gsim)

project(${NAME})

set(CMAKE_CXX_STANDARD          20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENABLE_UI "Build with GLFW + ImGui UI" ON)

add_subdirectory(external)
add_subdirectory(src/simulation) #gsim-simulation library
#add_subdirectory(src/renderer)
#add_subdirectory(src/compute)
#add_subdirectory(src/analasys)
#add_subdirectory(src/profiling)

#if(ENABLE_UI)
#    add_subdirectory(src/ui)
#endif()

add_subdirectory(tests)

add_executable(${NAME} src/main.cpp)

target_link_libraries(${NAME} PRIVATE
    gsim-simulation
)
