#include "scene/shape.h"

using namespace glm;

namespace rigid_sim {

float SignedVolumeOfTetrahedron(const vec3 &a, const vec3 &b, const vec3 &c) {
  return dot(a, cross(b, c)) / 6.0f;
}

void RMesh::ComputeCentroid() {
  float volumn = 0;
  centroid = vec3(0);
  for (auto face : _faces) {
    vec3 v1 = vec3(_verts[face.i0].x, _verts[face.i0].y, _verts[face.i0].z);
    vec3 v2 = vec3(_verts[face.i1].x, _verts[face.i1].y, _verts[face.i1].z);
    vec3 v3 = vec3(_verts[face.i2].x, _verts[face.i2].y, _verts[face.i2].z);
    float cur_volume = SignedVolumeOfTetrahedron(v1, v2, v3);
    centroid += cur_volume * (v1 + v2 + v3) / 4.0f;
    volumn += cur_volume;
  }
  centroid /= volumn;
  M = abs(volumn); // assume rho = 1
}



} // end namespace rigid_sim