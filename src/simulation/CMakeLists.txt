set(NAME gsim-simulation)

add_library(${NAME} STATIC ObjectView.cpp GravityEngine.cpp Tree.cpp Integrator.cpp)

target_include_directories(${NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${NAME} PUBLIC glm)


