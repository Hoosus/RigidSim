#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/common.hpp>
#include <luisa/luisa-compute.h>
#include <scene/scene.h>
#include <scene/shape.h>
#include <vector>

using namespace glm;

namespace rigid_sim {


void Scene::_simulation(vec3 gravity, float timestep) {
  printf("start simulation with timestep {%.3f}\n", timestep);
  
  Geometry &g = _geometry;
  std::vector<RMesh> &meshes = g.meshes();
  int N = meshes.size();
  if (N == 0) { return; }

  for (auto &mesh : meshes) if (!mesh.is_fixed) {
    // mesh.f = gravity * mesh.M; 
    mesh.f = vec3(0.f); 
    mesh.tau = vec3(0.0f); 
    mesh.omega = vec3(0.5f);
  }

  // TODO



  for (auto &mesh : meshes) if (!mesh.is_fixed) {
    glm::vec3 a = mesh.f / mesh.M;
    mesh.v = mesh.v + a * timestep;
    mesh.x = mesh.x + mesh.v * timestep;
    mesh.omega = mesh.omega + mesh.inv_I * mesh.tau;
    mesh.R = mesh.R + cross(glm::quat{0.0f, mesh.omega * timestep * 0.5f}, mesh.R);
    printf("update v = %.5f %.5f %.5f\n", mesh.v.x, mesh.v.y, mesh.v.z);
  }
}




} // end namespace rigid_sim