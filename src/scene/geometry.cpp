#include "scene/geometry.h"
#include "render/material.h"
#include "glm/glm.hpp"
#include <glm/gtx/quaternion.hpp>
#include <glm/common.hpp>

namespace rigid_sim {

Geometry::Geometry(Device &device, Stream &stream) :
       _device(device), _stream(stream) {}

Geometry::Geometry(Device &device, Stream &stream, std::vector<RMesh> shapes) :
       _device(device), _stream(stream), _shapes(shapes) {}

void Geometry::Build() {
  heapf = _device.create_bindless_array();
  heapv = _device.create_bindless_array();
  heapm = _device.create_bindless_array();
  luisa::vector<Mesh> meshes;
  triangle_buffers.clear();
  vertex_buffers.clear();
  material_buffers.clear();
  luisa::vector<RMaterial> materials;
  materials.resize(_shapes.size());
  Buffer<RMaterial> &material_buffer = material_buffers.emplace_back(_device.create_buffer<RMaterial>(_shapes.size()));

  for (auto shape: _shapes) {
    uint index = static_cast<uint>(meshes.size());
    auto &verts = shape.verts();
    auto &faces = shape.faces();
    uint triangle_count = faces.size();

    std::vector<float3> verts_transformed;
    verts_transformed.resize(verts.size());

    printf("find translate %.5lf %.5lf %.5lf\n", shape.x.x, shape.x.y, shape.x.z);
    printf("find rotate %.5lf %.5lf %.5lf %.5lf\n", shape.R.x, shape.R.y, shape.R.z, shape.R.w);

    glm::mat3 R_mat = glm::toMat3(shape.R);
    glm::vec3 trans = shape.x + shape.centroid;
    float3 translate = make_float3(trans.x, trans.y, trans.z); 
    // both are column-major
    float3x3 R_mat_luisa = make_float3x3(R_mat[0][0], R_mat[0][1], R_mat[0][2],
                                         R_mat[1][0], R_mat[1][1], R_mat[1][2],
                                         R_mat[2][0], R_mat[2][1], R_mat[2][2]);     

    printf("R %.5lf %.5lf %.5lf\n", R_mat[0][0], R_mat[0][1], R_mat[0][2]);
    printf("R %.5lf %.5lf %.5lf\n", R_mat[1][0], R_mat[1][1], R_mat[1][2]);
    printf("R %.5lf %.5lf %.5lf\n", R_mat[2][0], R_mat[2][1], R_mat[2][2]);

    for (int i = 0; i < verts.size(); ++i) {
      verts_transformed[i] = R_mat_luisa * verts[i] + translate;
    }

    // printf("%d %d\n", verts.size(), faces.size());
    // for (auto face: faces) {
    //   printf("%d %d %d\n", face.i0, face.i1, face.i2);
    // }

    Buffer<float3> &vertex_buffer = vertex_buffers.emplace_back(_device.create_buffer<float3>(verts.size()));
    Buffer<Triangle> &triangle_buffer = triangle_buffers.emplace_back(_device.create_buffer<Triangle>(faces.size()));
    auto &mesh = meshes.emplace_back(_device.create_mesh(vertex_buffer, triangle_buffer));
    heapf.emplace_on_update(index, triangle_buffer);
    heapv.emplace_on_update(index, vertex_buffer);
    _stream << vertex_buffer.copy_from(verts_transformed.data())
            << triangle_buffer.copy_from(faces.data())
            << mesh.build();
    materials[index] = shape.material();
  }
  heapm.emplace_on_update(0, material_buffer);
  _stream << material_buffer.copy_from(materials.data());

  _accel = _device.create_accel({});
  for (int i = 0; i < meshes.size(); ++i) {
    _accel.emplace_back(meshes[i]);
  }
  _stream << heapf.update()
          << heapv.update()
          << heapm.update()
          << _accel.build()
          << synchronize();
}

Var<TriangleHit> Geometry::intersect(const Var<Ray> &ray) {
 
  Var<TriangleHit> hit = _accel->intersect(ray, AccelTraceOptions{});
  return hit;
}

} // end namespace rigid_sim