#pragma once 

#include <vector>
#include <luisa/luisa-compute.h>
#include <render/material.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

using luisa::float3;
using luisa::compute::Triangle;
using luisa::make_float4x4;
using luisa::float4x4;

namespace rigid_sim {

static const uint RMESH_TAG_BOX = 1u;

class RMesh {
 public:
  RMesh(std::vector<float3> verts, std::vector<Triangle> faces, bool fixed=false) : _verts(verts), _faces(faces) {
    is_fixed = fixed;
    if (!is_fixed) {
      ComputeCentroid();
      for (auto &vert : _verts) {
        vert += make_float3(-centroid.x, -centroid.y, -centroid.z);
      }
    } else {
      centroid = glm::vec3(0);
      inv_I = glm::identity<glm::mat3>();
      M = 1.f;
    }
    x = glm::vec3(0);
    v = glm::vec3(0);
    f = glm::vec3(0);
    tau = glm::vec3(0);
    omega = glm::vec3(0);
    R = glm::identity<glm::quat>();
    tag = 0u;
  }
  RMesh(std::vector<float3> verts, std::vector<Triangle> faces, std::vector<glm::vec3> normals)
      : RMesh(verts, faces, true) { _normals = normals; }
  ~RMesh() = default;

 private:
  std::vector<float3> _verts;
  std::vector<Triangle> _faces;
  std::vector<glm::vec3> _normals; // only available with fixed env
  RMaterial _mat;

 public:
  bool is_fixed;

  float M;
  glm::vec3 centroid;
  glm::quat R;
  glm::mat3 inv_I;
  glm::vec3 x, v, f, tau;
  glm::vec3 omega;
  glm::vec3 temp; // temporary storage
  uint count; // tmeporary storage

  uint tag;

 public:
  inline void SetMaterial(const RMaterial &mat) { _mat = mat; }
  inline void SetMaterialColor(std::array<float, 3> color) { _mat.color = color; }
  inline void SetMaterialType(MAT type) { _mat.mtype = static_cast<uint>(type); }
  inline void SetFixed(bool f = true) { is_fixed = f; }

  void ComputeCentroid();

 public:
  [[nodiscard]] auto &verts() noexcept { return _verts; }
  [[nodiscard]] auto &faces() noexcept { return _faces; }
  [[nodiscard]] auto &normals() noexcept { return _normals; }
  [[nodiscard]] auto &material() noexcept { return _mat; }
};



} // end namespace rigid_sim