#pragma once

#include <luisa/luisa-compute.h>

using namespace luisa::compute;

namespace rigid_sim {

enum MAT {
  DIFFUSE,
  SPECULAR,
  GLOSSY,
  GLASS,
  LIGHT
};

struct alignas(16) RMaterial {
  std::array<float, 3> color{{1.f, 1.f, 1.f}};
  uint mtype{DIFFUSE};
};

static_assert(sizeof(RMaterial) == 16u, "RMaterial size mismatch");


} // end namespace rigid_sim


LUISA_STRUCT(rigid_sim::RMaterial, color, mtype) {
    [[nodiscard]] auto c() const noexcept {
        return luisa::compute::def<luisa::float3>(color);
    }
    [[nodiscard]] auto is_diffuse() const noexcept { return mtype == uint(rigid_sim::DIFFUSE); }
    [[nodiscard]] auto is_specular() const noexcept { return mtype == uint(rigid_sim::SPECULAR); }
    [[nodiscard]] auto is_glossy() const noexcept { return mtype == uint(rigid_sim::GLOSSY); }
    [[nodiscard]] auto is_glass() const noexcept { return mtype == uint(rigid_sim::GLASS); }
    [[nodiscard]] auto is_light() const noexcept { return mtype == uint(rigid_sim::LIGHT); }
};