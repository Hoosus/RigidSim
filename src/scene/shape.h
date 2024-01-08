#pragma once 

#include <vector>
#include <luisa/luisa-compute.h>
#include <render/material.h>
#include <ext/glm/glm/glm.hpp>
#include <ext/glm/glm/gtc/quaternion.hpp>

using luisa::float3;
using luisa::compute::Triangle;
using luisa::make_float4x4;
using luisa::float4x4;

namespace rigid_sim {

class RMesh {
 public:
  RMesh(std::vector<float3> verts, std::vector<Triangle> faces) : _verts(verts), _faces(faces) {
    _transform = make_float4x4(1.f);
    is_fixed = false;
    ComputeCentroid();
    a = glm::vec3(0);
    v = glm::vec3(0);
    omega = glm::vec3(0);
    R = glm::quat{};
  }
  ~RMesh() = default;

 private:
  float4x4 _transform;
  std::vector<float3> _verts;
  std::vector<Triangle> _faces;
  RMaterial _mat;

 public:
  bool is_fixed;

  float M;
  glm::vec3 centroid;
  glm::quat R;
  glm::mat3 I;
  glm::vec3 v, a;
  glm::vec3 omega;

 public:
  inline void SetMaterial(const RMaterial &mat) { _mat = mat; }
  inline void SetMaterialColor(std::array<float, 3> color) { _mat.color = color; }
  inline void SetMaterialType(MAT type) { _mat.mtype = static_cast<uint>(type); }
  inline void SetFixed(bool f = true) { is_fixed = f; }

  void ComputeCentroid();

 public:
  [[nodiscard]] auto &verts() noexcept { return _verts; }
  [[nodiscard]] auto &faces() noexcept { return _faces; }
  [[nodiscard]] auto &transform() noexcept { return _transform; }
  [[nodiscard]] auto &material() noexcept { return _mat; }
};



} // end namespace rigid_sim