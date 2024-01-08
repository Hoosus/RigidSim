set(RIGID_SIM_SIM_SOURCES
        sim.cpp sim.h)

add_library(rigid-sim-sim SHARED ${RIGID_SIM_SIM_SOURCES})
target_link_libraries(rigid-sim-sim PUBLIC luisa-shader-toy-ext luisa::compute)
