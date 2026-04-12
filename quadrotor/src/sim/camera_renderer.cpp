#include "sim/camera_renderer.hpp"

#include <algorithm>
#include <string>
#include <utility>

#include <GLFW/glfw3.h>

namespace quadrotor {
namespace {

constexpr int kMaxSceneGeoms = 100000;

bool HasGraphicalDisplay() {
#if defined(__linux__)
  return std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
#else
  return true;
#endif
}

void AssignError(std::string* error, std::string message) {
  if (error != nullptr) {
    *error = std::move(message);
  }
}

}  // namespace

CameraRenderer::~CameraRenderer() { FreeResources(); }

bool CameraRenderer::Initialize(mjModel* model, int max_width, int max_height, std::string* error) {
  return RefreshModel(model, max_width, max_height, error);
}

bool CameraRenderer::RefreshModel(mjModel* model, int max_width, int max_height, std::string* error) {
  if (model == nullptr) {
    AssignError(error, "camera renderer received a null model");
    return false;
  }
  if (!EnsureWindow(error)) {
    return false;
  }

  glfwMakeContextCurrent(window_);

  if (scene_initialized_) {
    mjv_freeScene(&scene_);
    scene_initialized_ = false;
  }
  if (context_initialized_) {
    mjr_freeContext(&context_);
    context_initialized_ = false;
  }

  model->vis.global.offwidth = std::max(model->vis.global.offwidth, max_width);
  model->vis.global.offheight = std::max(model->vis.global.offheight, max_height);

  mjv_defaultOption(&option_);
  mjv_defaultScene(&scene_);
  mjv_makeScene(model, &scene_, kMaxSceneGeoms);
  scene_initialized_ = true;

  mjr_defaultContext(&context_);
  mjr_makeContext(model, &context_, mjFONTSCALE_100);
  context_initialized_ = true;

  mjr_setBuffer(mjFB_OFFSCREEN, &context_);
  if (context_.currentBuffer != mjFB_OFFSCREEN) {
    AssignError(error, "MuJoCo offscreen framebuffer is unavailable");
    FreeResources();
    return false;
  }

  return true;
}

bool CameraRenderer::RenderRgb(const mjModel* model, mjData* data, int camera_id, int width, int height, std::vector<std::uint8_t>* rgb_pixels,
                               std::string* error) {
  if (model == nullptr || data == nullptr || rgb_pixels == nullptr) {
    AssignError(error, "camera renderer received invalid input");
    return false;
  }
  if (!available()) {
    AssignError(error, "camera renderer is not initialized");
    return false;
  }
  if (camera_id < 0) {
    AssignError(error, "camera id is invalid");
    return false;
  }
  if (width <= 0 || height <= 0) {
    AssignError(error, "camera image dimensions must be positive");
    return false;
  }
  if (width > context_.offWidth || height > context_.offHeight) {
    AssignError(error, "camera image dimensions exceed the offscreen framebuffer size");
    return false;
  }

  glfwMakeContextCurrent(window_);

  mjvCamera render_camera;
  mjv_defaultCamera(&render_camera);
  render_camera.type = mjCAMERA_FIXED;
  render_camera.fixedcamid = camera_id;
  render_camera.trackbodyid = -1;

  mjv_updateScene(model, data, &option_, nullptr, &render_camera, mjCAT_ALL, &scene_);
  mjr_setBuffer(mjFB_OFFSCREEN, &context_);

  const mjrRect viewport = {0, 0, width, height};
  mjr_render(viewport, &scene_, &context_);

  rgb_pixels->resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3u);
  mjr_readPixels(rgb_pixels->data(), nullptr, viewport, &context_);

  const std::size_t row_bytes = static_cast<std::size_t>(width) * 3u;
  for (int row = 0; row < height / 2; ++row) {
    auto top = rgb_pixels->begin() + static_cast<std::ptrdiff_t>(row) * static_cast<std::ptrdiff_t>(row_bytes);
    auto bottom = rgb_pixels->begin() + static_cast<std::ptrdiff_t>(height - 1 - row) * static_cast<std::ptrdiff_t>(row_bytes);
    std::swap_ranges(top, top + static_cast<std::ptrdiff_t>(row_bytes), bottom);
  }

  return true;
}

bool CameraRenderer::EnsureWindow(std::string* error) {
  if (window_ != nullptr) {
    return true;
  }
  if (!HasGraphicalDisplay()) {
    AssignError(error, "no graphical display detected for MuJoCo camera rendering");
    return false;
  }
  if (!glfwInit()) {
    AssignError(error, "could not initialize GLFW for MuJoCo camera rendering");
    return false;
  }

  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
  window_ = glfwCreateWindow(1, 1, "quadrotor_camera_renderer", nullptr, nullptr);
  if (window_ == nullptr) {
    AssignError(error, "could not create a hidden GLFW window for MuJoCo camera rendering");
    return false;
  }

  glfwMakeContextCurrent(window_);
  return true;
}

void CameraRenderer::FreeResources() {
  if (scene_initialized_) {
    mjv_freeScene(&scene_);
    scene_initialized_ = false;
  }
  if (context_initialized_) {
    mjr_freeContext(&context_);
    context_initialized_ = false;
  }
  if (window_ != nullptr) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
}

}  // namespace quadrotor
