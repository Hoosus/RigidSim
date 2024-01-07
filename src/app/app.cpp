#include <gui/shader_toy.h>
#include <luisa/luisa-compute.h>
#include <scene/scene.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <ext/tinyobjloader/tiny_obj_loader.h>


using namespace luisa;
using namespace luisa::compute;
using namespace rigid_sim;

rigid_sim::RMesh ReadObj(std::string inputfile) {
  tinyobj::ObjReaderConfig reader_config;
  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(inputfile, reader_config)) {
    if (!reader.Error().empty()) {
      std::cerr << "TinyObjReader: " << reader.Error();
    }
    exit(1);
  }

  if (!reader.Warning().empty()) {
    std::cout << "TinyObjReader: " << reader.Warning();
  }

  auto& attrib = reader.GetAttrib();
  auto& shapes = reader.GetShapes();
  // auto& materials = reader.GetMaterials(); 

  assert(shapes.size() == 1);

  int num_verts = attrib.vertices.size() / 3;
  int num_faces = shapes[0].mesh.num_face_vertices.size();

  std::vector<float3> verts; // three in a row is a vertex
  std::vector<Triangle> faces; // three in a row is a face, verts index starting from 0
  verts.resize(num_verts);
  faces.resize(num_faces);
  for (int i = 0; i < num_verts; ++i) {
    verts[i] = make_float3(attrib.vertices[3 * i], attrib.vertices[3 * i + 1], attrib.vertices[3 * i + 2]);
  }
  for (int i = 0; i < num_faces; ++i) {
    assert (shapes[0].mesh.num_face_vertices[i] == 3);
    faces[i] = Triangle(shapes[0].mesh.indices[3 * i].vertex_index, 
                        shapes[0].mesh.indices[3 * i + 1].vertex_index, 
                        shapes[0].mesh.indices[3 * i + 2].vertex_index);
  }
  return rigid_sim::RMesh{verts, faces};
}

int main(int argc, char *argv[]) {

  gui::ShaderToy toy{argc, argv};

  std::string inputfile = "../assets/bunny.obj";
  rigid_sim::RMesh mesh = ReadObj(inputfile);

  auto& device = toy.device();
  auto& stream = toy.stream();
  Scene scene{device, stream};
  uint2 size = toy.size();
  scene.set_camera(Camera(120, make_float2(size.x, size.y)));
  scene.set_camera_offset(make_float3(0.0, 0.0, -2.0));

  scene.AddMesh(mesh);

  scene.geometry().Build();
  toy.run([&](Float2 fragCoord, Float2 iResolution, Float time, Float4 cursor) noexcept {
    Var<Ray> ray = scene.generate_ray(fragCoord);
    Float3 res = make_float3(0.f);
    $for (depth, 1u) {
      auto it = scene.geometry().intersect(ray);
      //   uint inst;
      //   uint prim;
      //   float2 bary;
      //   float committed_ray_t;  
      $if (it->miss()) {
        res = make_float3(1.0f, 0.0f, 0.0f);
        $break;
      } $else {
        Var<Triangle> triangle = scene.geometry().heapf->buffer<Triangle>(it.inst).read(it.prim);
        // res = make_float3(triangle.i0 / 8.f, it.inst * 1.f, it.prim / 12.f);
        Float3 p0 = scene.geometry().heapv->buffer<float3>(it.inst).read(triangle.i0);
        Float3 p1 = scene.geometry().heapv->buffer<float3>(it.inst).read(triangle.i1);
        Float3 p2 = scene.geometry().heapv->buffer<float3>(it.inst).read(triangle.i2);
        Float3 p = triangle_interpolate(it.bary, p0, p1, p2);
        Float3 n = normalize(cross(p1 - p0, p2 - p0));
        Float cos_wo = dot(-ray->direction(), n);
        $if (cos_wo < 0.f) { 
          n = -n; 
          cos_wo = -cos_wo;
        };
        // res = n * 0.5f + 0.5f;
        // res = make_float3(it.bary, 0.0f);
        // res = make_float3(triangle.i0 / 8.f, triangle.i1 / 8.f, triangle.i2/8.f);
        res = make_float3(0.0f, 0.8f, 0.0f) * cos_wo;
      };
    };
    return res;
  });
}