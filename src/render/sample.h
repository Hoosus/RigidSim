#ifndef SAMPLE_H__
#define SAMPLE_H__

#include <luisa/luisa-compute.h>

using namespace luisa::compute;

namespace rigid_sim {

Callable tea = [](UInt v0, UInt v1) noexcept {
    UInt s0 = def(0u);
    for (uint n = 0u; n < 4u; n++) {
        s0 += 0x9e3779b9u;
        v0 += ((v1 << 4) + 0xa341316cu) ^ (v1 + s0) ^ ((v1 >> 5u) + 0xc8013ea4u);
        v1 += ((v0 << 4) + 0xad90777du) ^ (v0 + s0) ^ ((v0 >> 5u) + 0x7e95761eu);
    }
    return v0;
};

Callable construct_world_basis = [](const Float3 &n, Float3 &a1, Float3 &a2) noexcept {
  $if (n.z < 0.9f) {
    a1 = cross(n, make_float3(0.f, 0.f, 1.f));
  } $else {
    a1 = cross(n, make_float3(0.f, 1.f, 0.f));
  };
  a2 = cross(n, a1);
};

Callable to_world = [](const Float3 &n, const Float3 &a1, const Float3 &a2, const Float3 &local_v) noexcept{
  return n * local_v.z + a1 * local_v.x + a2 * local_v.y;
};

Callable cosine_sample_hemisphere = [](Float2 rn) {
  Float r = sqrt(rn.x);
  Float phi = 2.0f * constants::pi * rn.y;
  return make_float3(r * cos(phi), r * sin(phi), sqrt(1.0f-rn.x));
};

Callable my_reflect = [](Float3 v, Float3 n) {
  return dot(v, n) * 2.f * n - v;
};

Callable refract = [](Float3 v, Float3 n, Float eta) {
  Float cos_wi = dot(v, n);
  Float3 out_perp = eta * (-v + cos_wi * n);
  Float3 out_para = -sqrt(abs(1.0f - length_squared(out_perp))) * n;
  return normalize(out_para + out_perp);
};


} // end namespace rigid_sim

#endif