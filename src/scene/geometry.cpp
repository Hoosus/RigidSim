#include "scene/geometry.h"

namespace rigid_sim {

Geometry::Geometry(Device &device) : _device(device) {
  _accel = _device.create_accel({});
  // LUISA_INFO_WITH_LOCATION("Geometry built with {} triangles.", _triangle_count);
}

Geometry::~Geometry() = default;

auto Geometry::intersect(Stream &stream, const Var<Ray> &ray) {

  /* 
  TODO: add mesh to acceleration structure
  */
  stream << _accel.build(AccelBuildRequest::FORCE_BUILD);
  Var<SurfaceHit> hit = _accel->intersect(ray, AccelTraceOptions{});
  // ref: 
  // struct SurfaceHit {
  //   uint inst;
  //   uint prim;
  //   float2 bary;
  //   float committed_ray_t;
  // };
  return hit;
}

} // end namespace rigid_sim