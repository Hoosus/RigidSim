#include <gui/shader_toy.h>
#include <luisa/luisa-compute.h>
#include <scene/scene.h>
#include <render/sample.h>
#include <glm/glm.hpp>
#include <render/light.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>


using namespace luisa;
using namespace luisa::compute;
using namespace rigid_sim;
using glm::vec3;

rigid_sim::RMesh ReadEnvObj(std::string inputfile) {
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

  int num_verts = attrib.vertices.size() / 3;

  std::vector<float3> verts; // three in a row is a vertex
  verts.resize(num_verts);
  for (int i = 0; i < num_verts; ++i) {
    verts[i] = make_float3(attrib.vertices[3 * i],
                           attrib.vertices[3 * i + 1],
                           attrib.vertices[3 * i + 2]);
  }
  std::vector<Triangle> faces; // three in a row is a face, verts index starting from 0
  std::vector<vec3> normals;
  for (int i = 0; i < shapes.size(); ++i) {
    int num_faces = shapes[i].mesh.num_face_vertices.size();
    for (int j = 0; j < num_faces; ++j) {
      assert (shapes[i].mesh.num_face_vertices[j] == 3);
      faces.push_back(Triangle(shapes[i].mesh.indices[3 * j].vertex_index, 
                               shapes[i].mesh.indices[3 * j + 1].vertex_index, 
                               shapes[i].mesh.indices[3 * j + 2].vertex_index));
      assert (shapes[i].mesh.indices[3 * j].normal_index == shapes[i].mesh.indices[3 * j + 1].normal_index);
      assert (shapes[i].mesh.indices[3 * j].normal_index == shapes[i].mesh.indices[3 * j + 2].normal_index);
      auto index = 3 * shapes[i].mesh.indices[3 * j].normal_index;                      
      normals.push_back(glm::vec3(attrib.normals[index], attrib.normals[index + 1], attrib.normals[index + 2]));
    }
  }
  rigid_sim::RMesh mesh{verts, faces, normals, true};
  mesh.SetMaterialColor({1.f, 1.f, 1.f});
  mesh.SetMaterialType(rigid_sim::DIFFUSE);
  return mesh;
}


rigid_sim::RMesh ReadObj(std::string inputfile, bool use_normal=false, float zoom=1.0) {
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
    verts[i] = make_float3(attrib.vertices[3 * i] * zoom,
                           attrib.vertices[3 * i + 1] * zoom,
                           attrib.vertices[3 * i + 2] * zoom);
  }
  std::vector<vec3> normals;
  for (int i = 0; i < num_faces; ++i) {
    assert (shapes[0].mesh.num_face_vertices[i] == 3);
    faces[i] = Triangle(shapes[0].mesh.indices[3 * i].vertex_index, 
                        shapes[0].mesh.indices[3 * i + 1].vertex_index, 
                        shapes[0].mesh.indices[3 * i + 2].vertex_index);
    if (!use_normal) continue;
    assert (shapes[0].mesh.indices[3 * i].normal_index == shapes[0].mesh.indices[3 * i + 1].normal_index);
    assert (shapes[0].mesh.indices[3 * i].normal_index == shapes[0].mesh.indices[3 * i + 2].normal_index);
    auto index = 3 * shapes[0].mesh.indices[3 * i].normal_index;                      
    normals.push_back(glm::vec3(attrib.normals[index], attrib.normals[index + 1], attrib.normals[index + 2]));
  }
  rigid_sim::RMesh mesh{verts, faces, normals, false};
  mesh.SetMaterialColor({1.f, 1.f, 1.f});
  mesh.SetMaterialType(rigid_sim::DIFFUSE);
  return mesh;
}

int main(int argc, char *argv[]) {

  gui::ShaderToy toy{argc, argv};

  std::string envfile = "../assets/cornell.obj";
  rigid_sim::RMesh env = ReadEnvObj(envfile);
  std::string lightfile = "../assets/cornell_light.obj";
  rigid_sim::RMesh light = ReadEnvObj(lightfile);
  light.SetMaterialType(rigid_sim::LIGHT);
  light.SetMaterialColor({6.f, 6.f, 4.5f});

  std::string bunnyfile = "../assets/bunny_200.obj";
  std::string spherefile = "../assets/sphere.obj";
  std::string tallboxfile = "../assets/cornell_tallbox.obj";
  std::string shortboxfile = "../assets/cornell_shortbox.obj";

  rigid_sim::RMesh bunny  = ReadObj(bunnyfile);
  rigid_sim::RMesh sphere  = ReadObj(spherefile);
  rigid_sim::RMesh tallbox  = ReadObj(tallboxfile, true);
  tallbox.tag |= rigid_sim::RMESH_TAG_BOX;
  rigid_sim::RMesh shortbox  = ReadObj(shortboxfile, true);
  shortbox.tag |= rigid_sim::RMESH_TAG_BOX;

  auto& device = toy.device();
  auto& stream = toy.stream();
  Scene scene{device, stream};
  uint2 size = toy.size();
  // scene.set_camera(Camera(120, make_float2(size.x, size.y)));
  // scene.set_camera_offset(make_float3(0.0, 1.0, -4.0));
  // scene.set_gravity(glm::vec3(0.0f, -1.0f, 0.0f));
  scene.set_camera(Camera(23, make_float2(size.x, size.y)));
  scene.set_camera_offset(make_float3(0.0, 1.0, -6.8));
  scene.set_gravity(glm::vec3(0.0f, -1.0f, 0.0f));

  scene.AddMesh(light);
  scene.AddMesh(env);
  bunny.x = vec3(-0.2f, 1.5f, 0.0f);
  scene.AddMesh(bunny);
  tallbox.x = vec3(0.f, 0.1f, 0.0f);
  tallbox.SetMaterialType(rigid_sim::SPECULAR);
  scene.AddMesh(tallbox);
  shortbox.x = vec3(0.f, 0.2f, 0.0f);
  shortbox.SetMaterialType(rigid_sim::GLASS);
  scene.AddMesh(shortbox);


  toy.run(scene);
  // scene.geometry().Build();
  // toy.run(&scene, [&](Float2 fragCoord, Float2 iResolution, Float time, Float4 cursor) noexcept {
  //   Float3 res = make_float3(0.f);
  //   UInt state = tea(tea(UInt(fragCoord.x), UInt(fragCoord.y)), UInt(time));
  //   Float rx = lcg(state);
  //   Float ry = lcg(state);
  //   Float3 a1, a2, wi, wo, pp;
  //   Float cos_wi, cos_wo, pdf_bsdf, pdf_light;
  //   Bool flipped;
  //   $for (spp, 1u) {
  //     rx = lcg(state);
  //     ry = lcg(state);
  //     Var<Ray> ray = scene.generate_ray(fragCoord + make_float2(rx, ry) - 0.5f);
  //     Float3 throughput = make_float3(1.f);
  //     $for (depth, 10u) {
  //       auto it = scene.geometry().intersect(ray);
  //       reorder_shader_execution();
  //       //   uint inst;
  //       //   uint prim;
  //       //   float2 bary;
  //       //   float committed_ray_t;  
  //       $if (it->miss()) {
  //         Float3 env_color = make_float3(0.8f, 0.65f, 0.75f) + ray->direction() * make_float3(0.2f, 0.35f, 0.25f);
  //         res += throughput * env_color;
  //         $break;
  //       };
  //       Var<Triangle> triangle = scene.geometry().heapf->buffer<Triangle>(it.inst).read(it.prim);
  //       // res = make_float3(triangle.i0 / 8.f, it.inst * 1.f, it.prim / 12.f);
  //       Float3 p0 = scene.geometry().heapv->buffer<float3>(it.inst).read(triangle.i0);
  //       Float3 p1 = scene.geometry().heapv->buffer<float3>(it.inst).read(triangle.i1);
  //       Float3 p2 = scene.geometry().heapv->buffer<float3>(it.inst).read(triangle.i2);
  //       Float3 p = triangle_interpolate(it.bary, p0, p1, p2);
  //       Float3 n = normalize(cross(p1 - p0, p2 - p0));
  //       cos_wi = dot(-ray->direction(), n);

  //       $if (abs(cos_wi) < 1e-4f) { $break; };
  //       $if (cos_wi < 0.f) { 
  //         n = -n; 
  //         cos_wi = -cos_wi;
  //         flipped = true;
  //       } $else {
  //         flipped = false;
  //       };
  //       Var<RMaterial> mat = scene.geometry().heapm->buffer<RMaterial>(0).read(it.inst);
  //       // $if (mat->is_light()) {
  //       //   res += throughput * mat->c();
  //       //   $break;
  //       // };
  //       wi = -ray->direction();
  //       $if (mat->is_diffuse()) {          
  //         construct_world_basis(n, a1, a2);
  //         throughput *= mat->c();
  //         rx = lcg(state);
  //         ry = lcg(state);
  //         Float3 local_wo = cosine_sample_hemisphere(make_float2(rx, ry));
  //         cos_wo = local_wo.z;
  //         pdf_bsdf = cos_wo * inv_pi;
  //         wo = to_world(n, a1, a2, local_wo);
  //       } $elif (mat->is_glossy()) {
  //         // TODO
  //         res = make_float3(0.f, 0.f, 1.f);
  //         $break;
  //       } $elif (mat->is_specular()) {
  //         // TODO
  //         res = make_float3(0.f, 0.f, 1.f);
  //         $break;
  //         throughput *= mat->c();
  //         wo = my_reflect(wi, n);
  //       } $else {
  //         // TODO
  //         res = make_float3(0.f, 0.f, 1.f);
  //         $break;
  //         throughput *= mat->c();
  //         $if (flipped) {
  //           wo = refract(wi, n, 1.5f);
  //         } $else {
  //           wo = refract(wi, n, 0.67f);
  //         };
  //       };
  //       // $if (mat->is_glass()) {
  //       //   pp = offset_ray_origin(p, -n);
  //       // } $else {
  //       //   pp = offset_ray_origin(p, n);
  //       // };
  //       // pp = offset_ray_origin(p, wo);
  //       pp = p + (1.0f / 1024.f) * wo;
  //       ray = make_ray(pp, wo);
  //     };
  //   };
  //   return res / 1.f;
  // });
}

/*
plan:

blender搞一个场景，然后导出成obj，读进来，作为environment
其他的就直接丢就好了
然后实现刚体模拟部分

*/