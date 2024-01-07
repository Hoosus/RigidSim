#include <vector>
#include "luisa/luisa-compute.h"
#include "render/material.h"

using luisa::float3;
using luisa::compute::Triangle;
using luisa::make_float4x4;
using luisa::float4x4;

namespace rigid_sim {

class RMesh {
 public:
  RMesh(std::vector<float3> verts, std::vector<Triangle> faces) : _verts(verts), _faces(faces) {
    _transform = make_float4x4(1.f);
  }
  ~RMesh() = default;

 private:
  float4x4 _transform;
  std::vector<float3> _verts;
  std::vector<Triangle> _faces;
  RMaterial _mat;

 public:
  void SetMaterial(const RMaterial &mat) { _mat = mat; }
  void SetMaterialColor(float3 color) { _mat.color = color; }
  void SetMaterialType(MAT type) { _mat.mtype = static_cast<uint>(type); }

 public:
  [[nodiscard]] auto &verts() noexcept { return _verts; }
  [[nodiscard]] auto &faces() noexcept { return _faces; }
  [[nodiscard]] auto &transform() noexcept { return _transform; }
  [[nodiscard]] auto &material() noexcept { return _mat; }
};



} // end namespace rigid_sim