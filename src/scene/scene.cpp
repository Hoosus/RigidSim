#include "scene/scene.h"

namespace rigid_sim {

Scene::Scene(Device &device, Stream &stream) 
          : _device(device), _stream(stream), _geometry(device, stream) {}

Scene::Scene(Device &device, Stream &stream, std::vector<RMesh> &meshes) 
          : _device(device), _stream(stream), _geometry(device, stream, meshes) {}


} // end namespace rigid_sim