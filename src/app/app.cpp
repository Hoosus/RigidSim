#include <gui/shader_toy.h>
#include <luisa/luisa-compute.h>
#include <scene/scene.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <ext/tinyobjloader/tiny_obj_loader.h>


using namespace luisa;
using namespace luisa::compute;
using namespace rigid_sim;



int main(int argc, char *argv[]) {

    gui::ShaderToy toy{argc, argv};

    auto& stream = toy.stream();

    Scene scene{};

    toy.run([&](Float2 fragCoord, Float2 iResolution, Float time, Float4 cursor) noexcept {
        return make_float3(make_float2(fragCoord.xy() / 1000.f), 1.f);
    });
}