#ifndef SAMPLE_H__
#define SAMPLE_H__

#include <luisa/luisa-compute.h>

using namespace luisa::compute;

namespace rigid_sim {

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

Float3 cosine_sample_hemisphere(Float2 rn) {
  Float r = sqrt(rn.x);
  Float phi = 2.0f * constants::pi * rn.y;
  return make_float3(r * cos(phi), r * sin(phi), sqrt(1.0f-rn.x));
}

Float3 reflect(Float3 v, Float3 n) {
  return dot(v, n) * 2.f * n - v;
}

Float3 refract(Float3 v, Float3 n, Float eta) {
  Float cos_wi = dot(v, n);
  Float3 out_perp = eta * (-v + cos_wi * n);
  Float3 out_para = -sqrt(abs(1.0f - length_squared(out_perp))) * n;
  return normalize(out_para + out_perp);
}


} // end namespace rigid_sim

#endif