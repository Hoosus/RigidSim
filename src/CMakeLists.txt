include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

set(LUISA_COMPUTE_ENABLE_GUI OFF CACHE BOOL "" FORCE)
add_subdirectory(compute)

add_subdirectory(app)
add_subdirectory(ext)
add_subdirectory(gui)
add_subdirectory(render)
add_subdirectory(rigid)
add_subdirectory(scene)

add_library(luisa::shader-toy ALIAS luisa-shader-toy-gui)