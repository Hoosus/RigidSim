#pragma once 

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/common.hpp>
#include <luisa/luisa-compute.h>
#include <scene/scene.h>
#include <scene/shape.h>
#include <vector>
#include <render/material.h>

using namespace glm;

namespace rigid_sim {

typedef std::pair<vec3, vec3> AABB;

inline const vec3& lower(const AABB &x) { return x.first; }
inline const vec3& upper(const AABB &x) { return x.second; }

bool AABB_not_intersect(const AABB &a, const AABB &b) {
  if (lower(a).x > upper(b).x) return true;
  if (lower(b).x > upper(a).x) return true;
  if (lower(a).y > upper(b).y) return true;
  if (lower(b).y > upper(a).y) return true;
  if (lower(a).z > upper(b).z) return true;
  if (lower(b).z > upper(a).z) return true;
  return false;
}

inline vec3 gv(const luisa::float3 &x) { return vec3(x.x, x.y, x.z); }
inline int sgn(float x) { return x >= 0.f ? 1 : -1; }
inline vec3 xyz_min(const vec3 &x, const vec3 &y) { 
  return vec3(std::min(x.x, y.x), std::min(x.y, y.y), std::min(x.z, y.z));
}
inline vec3 xyz_max(const vec3 &x, const vec3 &y) { 
  return vec3(std::max(x.x, y.x), std::max(x.y, y.y), std::max(x.z, y.z));
}

inline mat3 mat_operator(vec3 r) {
  return mat3{0.f, r.z, -r.y, -r.z, 0.f, r.x, r.y, -r.x, 0.f};
}

void Scene::_simulation(vec3 gravity, float timestep) {
  float muN = 0.9; // normal direction, coefficient of bouncing back
  float muT = 0.8;  // tangent direction, coefficient of friction

  printf("start simulation with timestep {%.3f}\n", timestep);
  
  Geometry &g = _geometry;
  std::vector<RMesh> &meshes = g.meshes();
  int N = meshes.size();
  if (N == 0) { return; }

  // initialize and apply gravity
  for (auto &mesh : meshes) if (!mesh.is_fixed) {
    mesh.f = gravity * mesh.M; 
    mesh.tau = vec3(0.0f); 
  }

  std::vector<mat3> mat_rotate;
  std::vector<vec3> vec_trans;
  std::vector<AABB> AABBs;

  // move a step
  for (auto &mesh : meshes) if (!mesh.is_fixed) {
    glm::vec3 a = mesh.f / mesh.M;
    mesh.v = mesh.v + a * timestep;
    mesh.x = mesh.x + mesh.v * timestep;
    mat3 R_mat = toMat3(mesh.R);
    mesh.omega = mesh.omega + R_mat * mesh.inv_I * transpose(R_mat) * mesh.tau;
    mesh.R = mesh.R + cross(glm::quat{0.0f, mesh.omega * timestep * 0.5f}, mesh.R);
    // printf("update v = %.5f %.5f %.5f\n", mesh.v.x, mesh.v.y, mesh.v.z);
    
    mesh.f = vec3(0);
    mesh.tau = vec3(0);
    mesh.temp = vec3(0);
    mesh.count = 0u;
  }
   
  // find bounding box
  for (auto &mesh : meshes) {
    mat3 rotate = toMat3(mesh.R);
    vec3 trans = mesh.centroid + mesh.x;
    mat_rotate.push_back(rotate);
    vec_trans.push_back(trans);
    vec3 mn = vec3(1e10f);
    vec3 mx = vec3(-1e10f);
    for (auto vert : mesh.verts()) {
      vec3 actual_position = rotate * vec3(vert.x, vert.y, vert.z) + trans;
      mn = xyz_min(mn, actual_position);
      mx = xyz_max(mx, actual_position);
      // printf("find actual position %.5lf %.5lf %.5lf\n", actual_position.x, actual_position.y, actual_position.z);
    }
    AABBs.push_back(std::make_pair(mn, mx));
    // printf("mesh AABB mn = [%.2lf, %.2lf, %.2lf]\n", mn.x, mn.y, mn.z);
    // printf("mesh AABB mx = [%.2lf, %.2lf, %.2lf]\n", mx.x, mx.y, mx.z);
  }

  // collision detection and response
  for (int i = 0; i < N; ++i) if (!meshes[i].is_fixed) {
    for (int j = 0; j < N; ++j) {
      if (i == j) continue;
      if (AABB_not_intersect(AABBs[i], AABBs[j])) continue;
      if (meshes[j].material().mtype == rigid_sim::LIGHT) continue;
      if (meshes[j].is_fixed || (j > i)) {

        auto &verts1 = meshes[i].verts();
        auto &faces1 = meshes[i].faces();
        auto &verts2 = meshes[j].verts();
        auto &faces2 = meshes[j].faces();
        auto &normals2 = meshes[j].normals();

        for (int k = 0; k < verts1.size(); ++k) {
          vec3 u = gv(verts1[k]);
          vec3 X = mat_rotate[i] * u + vec_trans[i];
          printf("[%.3lf %.3lf %.3lf]\n", mat_rotate[i][0][0], mat_rotate[i][1][0], mat_rotate[i][2][0]);
          printf("[%.3lf %.3lf %.3lf]\n", mat_rotate[i][0][1], mat_rotate[i][1][1], mat_rotate[i][2][1]);
          printf("[%.3lf %.3lf %.3lf]\n", mat_rotate[i][0][2], mat_rotate[i][1][2], mat_rotate[i][2][2]);
          vec3 cur_f = vec3(0);
          vec3 cur_tau = vec3(0);
          vec3 cur_tau2 = vec3(0);
          vec3 cur_offset = vec3(0);
          int cur_count = 0;
          for (int l = 0; l < faces2.size(); ++l) {
            vec3 v1 = mat_rotate[j] * gv(verts2[faces2[l].i0]) + vec_trans[j];
            vec3 v2 = mat_rotate[j] * gv(verts2[faces2[l].i1]) + vec_trans[j];
            vec3 v3 = mat_rotate[j] * gv(verts2[faces2[l].i2]) + vec_trans[j];
            vec3 nv;
            if (normals2.size() > 0) {
              nv = normalize(normals2[l]);
            } else {
              printf("compute normal!\n");
              nv = normalize(cross(v2 - v1, v3 - v1));
            }
            float d_u1 = dot(X - v1, nv);
            if (d_u1 >= 0.0) continue;   
            if (!meshes[j].is_fixed && d_u1 < -0.1) continue;
            vec3 xp = X - v1 - d_u1 * nv;
            vec3 e2 = v2 - v1;
            vec3 e3 = v3 - v1;
            float d00 = dot(e2, e2);
            float d11 = dot(e3, e3);
            float d01 = dot(e2, e3);
            float d20 = dot(xp, e2);
            float d21 = dot(xp, e3);
            float denom = d00 * d11 - d01 * d01; 
            float alpha = (d11 * d20 - d01 * d21) / denom;
            float beta = (d00 * d21 - d01 * d20) / denom;
            // printf("alpha beta %.2lf %.2lf\n", alpha, beta);
            if (alpha < 0 || beta < 0 || alpha + beta > 1) continue;

            vec3 V = meshes[i].v + cross(meshes[i].omega, mat_rotate[i] * u) - meshes[j].v;
            printf("mesh %d verts %d: u[%.5lf %.5lf %.5lf] centroid[%.5lf %.5lf %.5lf] trans[%.5lf %.5lf %.5lf] X[%.5lf %.5lf %.5lf] inside mesh %d face %d normal = [%.5lf %.5lf %.5lf] distance = %.5lf relV=[%.3lf %.3lf %.3lf]\n", 
                    i, k, u.x, u.y, u.z, meshes[i].centroid.x, meshes[i].centroid.y, meshes[i].centroid.z, meshes[i].x.x, meshes[i].x.y, meshes[i].x.z, X.x, X.y, X.z, j, l, nv.x, nv.y, nv.z, d_u1, V.x, V.y, V.z);
            printf("v1 is [%.5lf %.5lf %.5lf], v2 is [%.5lf %.5lf %.5lf], v3 is [%.5lf %.5lf %.5lf]\n", v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, v3.y, v3.z);

            if (dot(V, nv) < 0) { // velocity into the environment
              vec3 vN = dot(V, nv) * nv;
              vec3 vT = V - vN;
              float a = std::max(0.f, 1.f - muT * (1 + muN) * length(vN) / length(vT));
              vec3 vN_new = -muN * vN;
              if (meshes[i].tag == 0) vN_new *= 0.5;
              vec3 vT_new = a * vT;
              vec3 v_new = vN_new + vT_new;
              printf("vnew=[%.2lf %.2lf %.2lf]\n", v_new.x, v_new.y, v_new.z);
              vec3 Rr = mat_rotate[i] * u;
              mat3 Rr_op = mat_operator(Rr);
              mat3 K = mat3{1.f / meshes[i].M} - Rr_op * meshes[i].inv_I * Rr_op;
              vec3 jj = inverse(K) * (v_new - V);
              cur_f += jj; // actually impulse, not force
              cur_tau += cross(Rr, jj);
              vec3 Rr2 = X - (vec_trans[j]);
              cur_tau2 += cross(Rr2, -jj);
              cur_offset -= nv * d_u1;
              cur_count += 1;
            }
          }
          if (cur_count > 0) {
            meshes[i].f += cur_f / float(cur_count);
            meshes[i].tau += cur_tau / float(cur_count);
            meshes[i].temp += cur_offset / float(cur_count);
            meshes[i].count += 1;
            if (!meshes[j].is_fixed) {
              meshes[j].f -= cur_f / float(cur_count);
              meshes[j].tau += cur_tau2 / float(cur_count);
              meshes[j].temp -= cur_offset / float(cur_count) * 0.5f;
              meshes[i].temp -= cur_offset / float(cur_count) * 0.5f;
              meshes[j].count += 1;
            }
          }
        }
      }
    }
  }
  for (auto &mesh : meshes) if (!mesh.is_fixed && mesh.count > 0) {
    mesh.f /= float(mesh.count);
    mesh.tau /= float(mesh.count);
    mesh.temp /= float(mesh.count);
    // printf("pre v = %.2lf %.2lf %.2lf\n", mesh.v.x, mesh.v.y, mesh.v.z);
    mesh.v += 1.f / mesh.M * mesh.f; // actually impulse, not force
    // printf("after v = %.2lf %.2lf %.2lf\n", mesh.v.x, mesh.v.y, mesh.v.z);
    printf("find temp %.3lf %.3lf %.3lf\n", mesh.temp.x, mesh.temp.y, mesh.temp.z);
    mesh.omega += mesh.inv_I * mesh.tau;
    mesh.x += mesh.temp;
    mesh.f = vec3(0);
    mesh.tau = vec3(0);
    mesh.temp = vec3(0);
  }
}




} // end namespace rigid_sim