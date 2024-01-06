#include <gui/shader_toy.h>
#include <luisa/luisa-compute.h>
#include <scene/scene.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <ext/tinyobjloader/tiny_obj_loader.h>


using namespace luisa;
using namespace luisa::compute;
using namespace rigid_sim;

void ReadObj(std::string inputfile) {
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

  int num_verts = attrib.vertices.size();
  int num_faces = shapes[0].mesh.num_face_vertices.size();

  std::vector<float> verts = attrib.vertices;
  std::vector<float> faces;
  faces.resize(num_faces * 3);
  for (int i = 0; i < num_faces; ++i) {
    faces[3 * i] = shapes[0].mesh.indices[i].vertex_index * 3;
    faces[3 * i + 1] = shapes[0].mesh.indices[i].vertex_index * 3 + 1;
    faces[3 * i + 2] = shapes[0].mesh.indices[i].vertex_index * 3 + 2;
  }

  // TODO
}

int main(int argc, char *argv[]) {

  gui::ShaderToy toy{argc, argv};

  std::string inputfile = "../assets/box.obj";
  ReadObj(inputfile);

  auto& stream = toy.stream();
  Scene scene{};

  toy.run([&](Float2 fragCoord, Float2 iResolution, Float time, Float4 cursor) noexcept {
    return make_float3(make_float2(fragCoord.xy() / 1000.f), 1.f);
  });
}