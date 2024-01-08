#pragma once 

#include "scene/geometry.h"
#include "render/camera.h"
#include <ext/glm/glm/glm.hpp>

namespace rigid_sim {

class Scene {
 public:
  Scene(Device &device, Stream &stream);
  Scene(Device &device, Stream &stream, std::vector<RMesh> &meshes);
  ~Scene() = default;

  void AddMesh(const RMesh &mesh) { _geometry.AddMesh(mesh); }
  Var<Ray> generate_ray(Float2 pixel) { return _camera.generate_ray_camera_space(_camera_offset, pixel); }
  void step(float time_step) { _simulation(_gravity, time_step); }

 private:
  Geometry _geometry;
  Device &_device;
  Stream &_stream;
  
  Camera _camera;
  float3 _camera_offset;

  glm::vec3 _gravity;

  void _simulation(glm::vec3 gravity, float time_step);

 public:
  inline void set_camera(Camera camera) { _camera = camera; }
  inline void set_camera_offset(float3 offset) { _camera_offset = offset; }
  inline void set_gravity(glm::vec3 gravity) { _gravity = gravity; }

  [[nodiscard]] auto &device() noexcept { return _device; }
  [[nodiscard]] auto &stream() noexcept { return _stream; }
  [[nodiscard]] auto &geometry() noexcept { return _geometry; }
  // [[nodiscard]] auto &camera() noexcept { return _camera; }
};


} // end namespace rigid_sim