set(RIGID_SIM_SOURCES
        scene.cpp scene.h
        geometry.cpp geometry.h
        shape.cpp shape.h)

add_library(rigid-sim-scene SHARED ${RIGID_SIM_SOURCES})
target_link_libraries(rigid-sim-scene PUBLIC luisa-shader-toy-ext luisa::compute)
# set_target_properties(luisa-shader-toy-gui PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
