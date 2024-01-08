#include "scene/shape.h"
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/inertia.h>
#include <vcg/complex/algorithms/create/platonic.h>

using namespace glm;

namespace rigid_sim {

float SignedVolumeOfTetrahedron(const vec3 &a, const vec3 &b, const vec3 &c) {
  return dot(a, cross(b, c)) / 6.0f;
}

/* see https://github.com/cnr-isti-vclab/vcglib/blob/edf834918fa5964096216a7d458eda08768d49ae/apps/sample/trimesh_create/trimesh_create.cpp */

class MyFace;
class MyVertex;

struct MyUsedTypes : public vcg::UsedTypes<	vcg::Use<MyVertex>::AsVertexType,    vcg::Use<MyFace>::AsFaceType>{};

class MyVertex  : public vcg::Vertex< MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::Color4b, vcg::vertex::BitFlags  >{};
class MyFace    : public vcg::Face < MyUsedTypes, vcg::face::VertexRef, vcg::face::Normal3f, vcg::face::FFAdj, vcg::face::BitFlags > {};
class MyMesh    : public vcg::tri::TriMesh< std::vector<MyVertex>, std::vector<MyFace> > {};

using namespace vcg;

// Note that this function is only valid for watertight mesh
// Not valid for environment, for example
void RMesh::ComputeCentroid() {
  // float volume = 0;
  // centroid = vec3(0);
  // for (auto face : _faces) {
  //   vec3 v1 = vec3(_verts[face.i0].x, _verts[face.i0].y, _verts[face.i0].z);
  //   vec3 v2 = vec3(_verts[face.i1].x, _verts[face.i1].y, _verts[face.i1].z);
  //   vec3 v3 = vec3(_verts[face.i2].x, _verts[face.i2].y, _verts[face.i2].z);
  //   float cur_volume = SignedVolumeOfTetrahedron(v1, v2, v3);
  //   centroid += cur_volume * (v1 + v2 + v3) / 4.0f;
  //   volume += cur_volume;
  // }
  // centroid /= volume;
  // M = abs(volume); // assume rho = 1
  // printf("pre algorithm compute: centroid {%0.2f, %0.2f, %0.2f}, volume={%.2f}\n", centroid.x, centroid.y, centroid.z, volume);

  MyMesh mesh;
  std::vector<Point3f> coordVec;
  std::vector<Point3i> indexVec;
  for (auto vert: _verts) {
    coordVec.push_back(Point3f(vert.x, vert.y, vert.z));
  }
  for (auto face: _faces) {
    indexVec.push_back(Point3i(face.i0, face.i1, face.i2));
  }
  tri::BuildMeshFromCoordVectorIndexVector(mesh, coordVec, indexVec);
  tri::Inertia<MyMesh> inertia{mesh};
  vcg::Matrix33f it;
  Point3f cc = inertia.CenterOfMass();
  centroid = vec3(cc[0], cc[1], cc[2]);
  M = inertia.Mass();
  inertia.InertiaTensor(it);
  // printf("vcg algorithm compute: centroid {%0.2f, %0.2f, %0.2f}, volume={%.2f}\n", cc[0], cc[1], cc[2], M);
  printf(" %6.3f %6.3f %6.3f\n",it[0][0], it[0][1], it[0][2]);
  printf(" %6.3f %6.3f %6.3f\n",it[1][0], it[1][1], it[1][2]);
  printf(" %6.3f %6.3f %6.3f\n",it[2][0], it[2][1], it[2][2]);
  glm::mat3 I{it[0][0], it[1][0], it[2][0], it[0][1], it[1][1], it[2][1], it[0][2], it[1][2], it[2][2]};
  inv_I = inverse(I);
}



} // end namespace rigid_sim