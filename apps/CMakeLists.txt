project(pibt)

add_executable(pibt_engine main.cpp)
target_link_libraries(pibt_engine PRIVATE graph pibt ${OPENGL_LIBRARIES} benchmark)