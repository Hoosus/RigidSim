#include <cmath>
#include <luisa/luisa-compute.h>

using namespace luisa::compute;

namespace rigid_sim {


class Camera {
 public:
  Camera(float fov=120, float2 resolution=make_float2(400, 400)) 
      : _tan_half_fov(std::tan(fov * 0.5 / 180 * pi)), _res(resolution) {}
  ~Camera() = default;

  Var<Ray> generate_ray_camera_space(Float3 origin, Float2 pixel) {
    auto p = (pixel * 2.0f - _res) * (_tan_half_fov / _res.y);
    auto direction = normalize(make_float3(p.x, p.y, 1.f));
    auto ray = make_ray(origin, direction);
    return ray;
  }

 private:
  float _tan_half_fov;
  float2 _res;
};


} // end namespace rigid_sim