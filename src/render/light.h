#pragma once

#include <luisa/luisa-compute.h>
#include <scene/scene.h>
#include <render/material.h>

namespace rigid_sim {

// Only square light
struct alignas(16) LightInfo {
  float3 light_position {make_float3(0.f)};
  float3 light_u {make_float3(0.f)};
  float3 light_v {make_float3(0.f)};
  float light_area {0.f};
  float3 color {make_float3(0.f)};
  bool valid {false};
};

LightInfo ConstructLightinfo(Scene &scene) {
  LightInfo info;
  info.valid = false;
  auto &geometry = scene.geometry();
  auto &meshes = geometry.meshes();
  for (auto &mesh : meshes) {
    if (mesh.material().mtype == LIGHT) {
      /*
      v 0.240000 1.980000 0.220000
      v -0.230000 1.980000 0.220000
      v 0.240000 1.980000 -0.160000
      v -0.230000 1.980000 -0.160000
      */
      info.light_position = make_float3(-0.23f + mesh.x.x, 1.98f + mesh.x.y, -0.16f + mesh.x.z);
      info.light_u = make_float3(0.47f, 0.f, 0.f);
      info.light_v = make_float3(0.f, 0.f, 0.38f);
      info.light_area = 0.47f * 0.38f;
      info.color = make_float3(mesh.material().color[0], mesh.material().color[1], mesh.material().color[2]);
      info.valid = true;
      return info;
    }
  }
  return info;
}


} // end namespace rigid_sim
