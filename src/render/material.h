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

struct RMaterial {
  float3 color{make_float3(1.f)};
  uint mtype{DIFFUSE};
};



} // end namespace rigid_sim


LUISA_STRUCT(rigid_sim::RMaterial, color, mtype) {
    [[nodiscard]] Float3 c() const noexcept {
        return make_float3(color);
    }
    [[nodiscard]] auto is_diffuse() const noexcept { return mtype == uint(rigid_sim::DIFFUSE); }
    [[nodiscard]] auto is_specular() const noexcept { return mtype == uint(rigid_sim::SPECULAR); }
    [[nodiscard]] auto is_glossy() const noexcept { return mtype == uint(rigid_sim::GLOSSY); }
    [[nodiscard]] auto is_glass() const noexcept { return mtype == uint(rigid_sim::GLASS); }
    [[nodiscard]] auto is_light() const noexcept { return mtype == uint(rigid_sim::LIGHT); }
};