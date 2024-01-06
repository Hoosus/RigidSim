#include "scene/shape.h"
#include <luisa/luisa-compute.h>

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
  Geometry(Device &device);
  ~Geometry();

 public:
  [[nodiscard]] auto intersect(Stream &stream, const Var<Ray> &ray);
  
 private:
  Device &_device;
  Accel _accel;

 public:
  [[nodiscard]] auto &device() noexcept { return _device; }
};



} // end namespace rigid_sim