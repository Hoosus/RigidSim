set(RIGID_SIM_RENDER
        camera.cpp camera.h
        material.cpp material.h
        sample.h
        light.h light.cpp)

add_library(rigid-sim-render SHARED ${RIGID_SIM_RENDER})
target_link_libraries(rigid-sim-render PUBLIC luisa-shader-toy-ext luisa::compute)
# set_target_properties(luisa-shader-toy-gui PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
