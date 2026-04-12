#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

struct GLFWwindow;

namespace quadrotor {

class CameraRenderer {
 public:
  CameraRenderer() = default;
  ~CameraRenderer();

  bool Initialize(mjModel* model, int max_width, int max_height, std::string* error);
  bool RefreshModel(mjModel* model, int max_width, int max_height, std::string* error);
  bool RenderRgb(const mjModel* model, mjData* data, int camera_id, int width, int height, std::vector<std::uint8_t>* rgb_pixels, std::string* error);
  bool available() const { return window_ != nullptr && context_initialized_ && scene_initialized_; }

 private:
  bool EnsureWindow(std::string* error);
  void FreeResources();

  GLFWwindow* window_ = nullptr;
  mjvScene scene_{};
  mjvOption option_{};
  mjrContext context_{};
  bool scene_initialized_ = false;
  bool context_initialized_ = false;
};

}  // namespace quadrotor
