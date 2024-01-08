//
// Created by Mike Smith on 2021/6/27.
//

#include <gui/shader_toy.h>
#include <scene/scene.h>
#include <render/sample.h>
#include <glm/glm.hpp>

#if LUISA_SHADERTOY_HAS_OPENCV
#include <opencv2/opencv.hpp>
#endif

using namespace luisa;
using namespace luisa::compute;
using namespace rigid_sim;
using glm::vec3;

namespace luisa::gui {

using namespace compute;

template<typename F>
static void with_panel(const char *name, F &&f) {
    ImGui::Begin(name, nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoResize);
    f();
    ImGui::End();
}

void ShaderToy::run(rigid_sim::Scene &scene) noexcept {
  if (_dump_file.empty()) {
    assert (false);
    _run_display(scene);
  } else {
    _run_dump(scene);
  }
}

ShaderToy::ShaderToy(int argc, const char *const *argv) noexcept
    : _context{argv[0]} {
    Context context{argv[0]};
    luisa::string backend{"unknown"};
    auto device_id = 0u;
    for (auto i = 1u; i < argc; i++) {
        using namespace std::string_view_literals;
        auto next_arg = [&] {
            if (i + 1u >= argc) {
                LUISA_ERROR_WITH_LOCATION(
                    "Missing argument for option: ",
                    argv[i]);
            }
            return argv[++i];
        };
        if (argv[i] == "-b"sv || argv[i] == "--backend"sv) {
            backend = next_arg();
        } else if (argv[i] == "-s"sv || argv[i] == "--size"sv) {
            auto s = next_arg();
            auto n = std::sscanf(s, "%ux%u", &_size.x, &_size.y);
            LUISA_ASSERT(n != 0, "Invalid size: {}", s);
            if (n == 1) { _size.y = _size.x; }
            _size = luisa::clamp(_size, 1u, 4096u);
        } else if (argv[i] == "-d"sv || argv[i] == "--device"sv) {
            device_id = std::atoi(next_arg());
        } else if (argv[i] == "-t"sv || argv[i] == "--step"sv) {
            _step = std::clamp(std::atof(next_arg()), 0., 1000.);
        } else if (argv[i] == "-o"sv || argv[i] == "--dump"sv) {
            _dump_file = next_arg();
        } else if (argv[i] == "-n"sv || argv[i] == "--frames"sv) {
            _dump_frames = std::atoi(next_arg());
        } else if (argv[i] == "--fps"sv) {
            _dump_fps = std::clamp(std::atof(next_arg()), 1., 200.);
        } else {
            LUISA_ERROR_WITH_LOCATION("Unknown option: {}", argv[i]);
        }
    }
    _title = std::filesystem::canonical(argv[0]).filename().replace_extension("").string();
    for (auto &c : _title) { c = c == '_' ? ' ' : c; }
    auto is_first = true;
    for (auto &c : _title) {
        if (is_first) { c = static_cast<char>(std::toupper(c)); }
        is_first = c == ' ';
    }
    compute::DeviceConfig config;
    config.device_index = 0;
    config.inqueue_buffer_limit = false;
    _device = luisa::make_unique<Device>(context.create_device(backend, &config));
    _stream = _device->create_stream();
}

void ShaderToy::_run_display(rigid_sim::Scene &scene) noexcept {
    assert(false);
    // auto device_image = _device->create_image<float>(PixelStorage::BYTE4, _size);
    // auto event = _device->create_event();
    // Window window{_title, _size};
    // GLTexture texture{PixelFormat::RGBA8UNorm, _size};
    // _stream << event.signal();

    // auto prev_key_up = false;
    // auto show_console = true;
    // auto cursor = float4(0.0f);
    // auto dragging = false;
    // Framerate framerate{0.8};
    // window.run([&] {
    //     auto render_size = device_image.size();
    //     auto window_size = window.size();
    //     if (window.mouse_down(MOUSE_LEFT)) {
    //         auto curr = window.cursor();
    //         curr = float2(curr.x, static_cast<float>(window_size.y) - curr.y);
    //         if (dragging) {
    //             cursor = make_float4(curr, cursor.zw());
    //         } else {
    //             cursor = make_float4(curr, curr * float2(1.0f, -1.0f));
    //             dragging = true;
    //         }
    //     } else if (window.mouse_up(MOUSE_LEFT)) {
    //         cursor = make_float4(cursor.xy(), -abs(cursor.zw()));
    //         dragging = false;
    //     }

    //     auto time = _step == 0. ? window.time() : static_cast<double>(framerate.count()) * _step;
    //     if (texture.present([&](void *pixels) noexcept {
    //             event.synchronize();
    //             _stream << shader(device_image, static_cast<float>(time), cursor).dispatch(window_size)
    //                     << device_image.copy_to(pixels)
    //                     << event.signal();
    //         })) {
    //         ImVec2 background_size{static_cast<float>(render_size.x), static_cast<float>(render_size.y)};
    //         ImGui::GetBackgroundDrawList()->AddImage(reinterpret_cast<ImTextureID *>(texture.handle()), {}, background_size);
    //     }

    //     framerate.tick();
    //     auto fps = framerate.fps();
    //     auto spp = framerate.count();
    //     if (show_console) {
    //         with_panel("Console", [&] {
    //             ImGui::Text("Frame: %llu", static_cast<uint64_t>(spp));
    //             ImGui::Text("Time:  %.2lfs", time);
    //             ImGui::Text("FPS:   %.1lf", fps);
    //             ImGui::Text("Size:  %ux%u", window_size.x, window_size.y);
    //         });
    //     }
    //     if (window.key_down(KEY_ESCAPE)) {
    //         window.notify_close();
    //     }
    //     if (prev_key_up && (window.key_down(KEY_LEFT_CONTROL) || window.key_down(KEY_RIGHT_CONTROL))) {
    //         show_console = !show_console;
    //     }
    //     prev_key_up = window.key_up(KEY_LEFT_CONTROL) && window.key_up(KEY_RIGHT_CONTROL);
    // });
}

void ShaderToy::_run_dump(rigid_sim::Scene &scene) noexcept {
#if LUISA_SHADERTOY_HAS_OPENCV
  auto device_image = _device->create_image<float>(PixelStorage::BYTE4, _size);
  cv::Size size{static_cast<int>(_size.x), static_cast<int>(_size.y)};
  cv::Mat frame{size, CV_8UC4, cv::Scalar::all(0)};
  cv::Mat cvt_frame{size, CV_8UC3, cv::Scalar::all(0)};
  auto fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
  auto path = std::filesystem::absolute(_dump_file);
  cv::VideoWriter video{path.string(), fourcc, _dump_fps, size};
  LUISA_ASSERT(video.isOpened(), "Failed to open video file: {}", path.string());
  for (auto i = 0u; i < _dump_frames; i++) {    
    scene.geometry().Build();
    Callable lcg = [](UInt &state) noexcept {
    constexpr uint lcg_a = 1664525u;
    constexpr uint lcg_c = 1013904223u;
    state = lcg_a * state + lcg_c;
    return cast<float>(state & 0x00ffffffu) *
                (1.0f / static_cast<float>(0x01000000u));
    };
    auto main_shader = [&](Float2 fragCoord, Float2 iResolution, Float time, Float4 cursor) noexcept {
      Float3 res = make_float3(0.f);
      UInt state = tea(tea(UInt(fragCoord.x), UInt(fragCoord.y)), UInt(time));
      Float rx = lcg(state);
      Float ry = lcg(state);
      Float3 a1, a2, wi, wo, pp;
      Float cos_wi, cos_wo, pdf_bsdf, pdf_light;
      Bool flipped;
      $for (spp, 10u) {
        rx = lcg(state);
        ry = lcg(state);
        Var<Ray> ray = scene.generate_ray(fragCoord + make_float2(rx, ry) - 0.5f);
        Float3 throughput = make_float3(1.f);
        $for (depth, 10u) {
          auto it = scene.geometry().intersect(ray);
          reorder_shader_execution();
          //   uint inst;
          //   uint prim;
          //   float2 bary;
          //   float committed_ray_t;  
          $if (it->miss()) {
            Float3 env_color = make_float3(0.8f, 0.65f, 0.75f) + ray->direction() * make_float3(0.2f, 0.35f, 0.25f);
            res += throughput * env_color;
            $break;
          };
          Var<Triangle> triangle = scene.geometry().heapf->buffer<Triangle>(it.inst).read(it.prim);
          // res = make_float3(triangle.i0 / 8.f, it.inst * 1.f, it.prim / 12.f);
          Float3 p0 = scene.geometry().heapv->buffer<float3>(it.inst).read(triangle.i0);
          Float3 p1 = scene.geometry().heapv->buffer<float3>(it.inst).read(triangle.i1);
          Float3 p2 = scene.geometry().heapv->buffer<float3>(it.inst).read(triangle.i2);
          Float3 p = triangle_interpolate(it.bary, p0, p1, p2);
          Float3 n = normalize(cross(p1 - p0, p2 - p0));
          cos_wi = dot(-ray->direction(), n);

          $if (abs(cos_wi) < 1e-4f) { $break; };
          $if (cos_wi < 0.f) { 
            n = -n; 
            cos_wi = -cos_wi;
            flipped = true;
          } $else {
            flipped = false;
          };
          Var<RMaterial> mat = scene.geometry().heapm->buffer<RMaterial>(0).read(it.inst);
          // $if (mat->is_light()) {
          //   res += throughput * mat->c();
          //   $break;
          // };
          wi = -ray->direction();
          $if (mat->is_diffuse()) {          
            construct_world_basis(n, a1, a2);
            throughput *= mat->c();
            rx = lcg(state);
            ry = lcg(state);
            Float3 local_wo = cosine_sample_hemisphere(make_float2(rx, ry));
            cos_wo = local_wo.z;
            pdf_bsdf = cos_wo * inv_pi;
            wo = to_world(n, a1, a2, local_wo);
          } $elif (mat->is_glossy()) {
            // TODO
            res = make_float3(0.f, 0.f, 1.f);
            $break;
          } $elif (mat->is_specular()) {
            // TODO
            res = make_float3(0.f, 0.f, 1.f);
            $break;
            throughput *= mat->c();
            wo = my_reflect(wi, n);
          } $else {
            // TODO
            res = make_float3(0.f, 0.f, 1.f);
            $break;
            throughput *= mat->c();
            $if (flipped) {
              wo = refract(wi, n, 1.5f);
            } $else {
              wo = refract(wi, n, 0.67f);
            };
          };
          // $if (mat->is_glass()) {
          //   pp = offset_ray_origin(p, -n);
          // } $else {
          //   pp = offset_ray_origin(p, n);
          // };
          // pp = offset_ray_origin(p, wo);
          pp = p + (1.0f / 1024.f) * wo;
          ray = make_ray(pp, wo);
        };
      };
      return res / 10.f;
    };
    auto shader = _device->compile(Kernel2D{[&](ImageFloat image, Float time, Float4 cursor) noexcept {
      using namespace compute;
      auto xy = dispatch_id().xy();
      auto resolution = dispatch_size().xy();
      auto col = main_shader(make_float2(make_uint2(xy.x, resolution.y - 1u - xy.y)) + 0.5f,
                              make_float2(resolution), time, cursor);
      image.write(xy, make_float4(col, 1.0f));
    }});
    _stream << shader(device_image, static_cast<float>(i * _step), float4(0.0f)).dispatch(_size)
            << device_image.copy_to(frame.data)
            << synchronize();
    LUISA_INFO("Frame {} / {}", i + 1u, _dump_frames);
    cv::cvtColor(frame, cvt_frame, cv::COLOR_RGBA2BGR);
    video << cvt_frame;
    if (i + 1 < _dump_frames) {
      scene.step(_step);
    }
  }
#else
  LUISA_WARNING("OpenCV is not available. Dumping is disabled.");
#endif
}

}// namespace luisa::gui
