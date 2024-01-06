function(luisa_shader_toy_add_executable name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE luisa::shader-toy)
endfunction()

luisa_shader_toy_add_executable(app)
