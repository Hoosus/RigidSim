#include "scene/geometry.h"

namespace rigid_sim {

Geometry::Geometry(Device &device, Stream &stream) :
       _device(device), _stream(stream) {}

Geometry::Geometry(Device &device, Stream &stream, std::vector<RMesh> shapes) :
       _device(device), _stream(stream), _shapes(shapes) {}

void Geometry::Build() {
  heapf = _device.create_bindless_array();
  heapv = _device.create_bindless_array();
  luisa::vector<Mesh> meshes;
  luisa::vector<float4x4> transforms;
  triangle_buffers.clear();
  vertex_buffers.clear();

  for (auto shape: _shapes) {
    uint index = static_cast<uint>(meshes.size());
    auto &verts = shape.verts();
    auto &faces = shape.faces();
    uint triangle_count = faces.size();

    printf("%d %d\n", verts.size(), faces.size());
    for (auto face: faces) {
      printf("%d %d %d\n", face.i0, face.i1, face.i2);
    }

    Buffer<float3> &vertex_buffer = vertex_buffers.emplace_back(_device.create_buffer<float3>(verts.size()));
    Buffer<Triangle> &triangle_buffer = triangle_buffers.emplace_back(_device.create_buffer<Triangle>(faces.size()));
    auto &mesh = meshes.emplace_back(_device.create_mesh(vertex_buffer, triangle_buffer));
    transforms.emplace_back(shape.transform());
    heapf.emplace_on_update(index, triangle_buffer);
    heapv.emplace_on_update(index, vertex_buffer);
    _stream << vertex_buffer.copy_from(verts.data()) << triangle_buffer.copy_from(faces.data()) << mesh.build();
  }

  _accel = _device.create_accel({});
  for (int i = 0; i < meshes.size(); ++i) {
    _accel.emplace_back(meshes[i], transforms[i]);
  }
  _stream << heapf.update()
          << heapv.update()
          << _accel.build()
          << synchronize();
}

Var<TriangleHit> Geometry::intersect(const Var<Ray> &ray) {
 
  Var<TriangleHit> hit = _accel->intersect(ray, AccelTraceOptions{});
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