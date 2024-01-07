#include "scene/shape.h"
#include <luisa/luisa-compute.h>
#include <vector>

using namespace luisa::compute;

// namespace rigid_sim {
// struct Hit {
//   uint inst;
//   uint prim;
//   float2 bary;
// };


// } // end namespace rigid_sim

// // clang-format off
// LUISA_STRUCT(rigid_sim::Hit, inst, prim, bary) {
//     [[nodiscard]] auto miss() const noexcept { return inst == ~0u; }
// };

namespace rigid_sim {

class Geometry {
 public:
  Geometry(Device &device, Stream &stream);
  Geometry(Device &device, Stream &stream, std::vector<RMesh> shapes);
  ~Geometry() = default;

 public:
  void AddMesh(const RMesh &mesh) { _shapes.push_back(mesh); }
  void Build();
  [[nodiscard]] Var<TriangleHit> intersect(const Var<Ray> &ray);
  
 private:
  Accel _accel;
  Device &_device;
  Stream &_stream;
  std::vector<RMesh> _shapes;

 public:
  BindlessArray heapf, heapv, heapm;
  luisa::vector<Buffer<Triangle>> triangle_buffers;
  luisa::vector<Buffer<float3>> vertex_buffers;

 public:
  [[nodiscard]] auto &device() noexcept { return _device; }
};



} // end namespace rigid_sim