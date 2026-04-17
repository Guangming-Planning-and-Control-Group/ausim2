#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <termios.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace remote_control {

class TerminalKeyboard {
 public:
  enum class Action {
    kTakeoff,
    kReset,
    kModeNext,
    kEmergencyStop,
  };

  explicit TerminalKeyboard(bool enabled, double key_timeout_seconds);
  ~TerminalKeyboard();

  TerminalKeyboard(const TerminalKeyboard&) = delete;
  TerminalKeyboard& operator=(const TerminalKeyboard&) = delete;

  bool available() const { return available_.load(); }

  geometry_msgs::msg::Twist BuildTwist(double linear_x_scale, double linear_y_scale, double linear_z_scale, double angular_yaw_scale) const;
  std::optional<Action> ConsumeAction();

 private:
  using TimePoint = std::chrono::steady_clock::time_point;

  void ReaderLoop();
  void HandleChar(char value);
  void ClearMovementLocked();

  struct KeyState {
    TimePoint forward_until{};
    TimePoint backward_until{};
    TimePoint left_until{};
    TimePoint right_until{};
    TimePoint up_until{};
    TimePoint down_until{};
    TimePoint yaw_left_until{};
    TimePoint yaw_right_until{};
  };

  std::chrono::steady_clock::duration key_timeout_{};
  mutable std::mutex mutex_;
  std::deque<Action> pending_actions_;
  KeyState key_state_;
  std::thread reader_thread_;
  std::atomic_bool running_{false};
  std::atomic_bool available_{false};
  bool terminal_configured_ = false;
  bool had_termios_ = false;
  struct termios* original_termios_ = nullptr;
};

class RemoteControlNode : public rclcpp::Node {
 public:
  RemoteControlNode();
  ~RemoteControlNode() override = default;

 private:
  enum class InputMode {
    kNone,
    kJoystick,
    kKeyboard,
  };

  void LoadParameters();
  void OnJoy(const sensor_msgs::msg::Joy::SharedPtr message);
  void OnPublishTimer();
  void TriggerAction(TerminalKeyboard::Action action, const char* source);
  void CallTriggerService(const std::string& label, const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client);
  void PublishZeroCommand();
  void UpdateInputMode(InputMode mode);
  geometry_msgs::msg::Twist BuildJoyTwist(const sensor_msgs::msg::Joy& message) const;
  bool ButtonPressed(const sensor_msgs::msg::Joy& message, int button_index) const;
  bool ButtonsPressed(const sensor_msgs::msg::Joy& message, const std::vector<int>& button_indices) const;
  double AxisValue(const sensor_msgs::msg::Joy& message, int axis_index, double scale) const;
  static std::vector<int> ToIntVector(const std::vector<std::int64_t>& values);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr teleop_event_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr takeoff_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<TerminalKeyboard> keyboard_;

  std::string joy_topic_;
  std::string cmd_vel_topic_;
  std::string teleop_event_topic_;
  std::string takeoff_service_;
  std::string reset_service_;
  double publish_rate_hz_ = 30.0;
  double joy_timeout_seconds_ = 0.5;
  double deadzone_ = 0.1;
  bool require_enable_button_ = false;
  int enable_button_ = 4;
  int axis_linear_x_ = 1;
  int axis_linear_y_ = 0;
  int axis_linear_z_ = 4;
  int axis_angular_yaw_ = 3;
  double scale_linear_x_ = 0.6;
  double scale_linear_y_ = -0.6;
  double scale_linear_z_ = 0.5;
  double scale_angular_yaw_ = -1.0;
  std::vector<int> takeoff_buttons_;
  std::vector<int> reset_buttons_;
  std::vector<int> mode_next_buttons_;
  std::vector<int> emergency_stop_buttons_;
  double command_cooldown_seconds_ = 0.5;
  double keyboard_key_timeout_seconds_ = 0.25;
  bool keyboard_enabled_ = true;
  double keyboard_scale_linear_x_ = 0.6;
  double keyboard_scale_linear_y_ = 0.6;
  double keyboard_scale_linear_z_ = 0.5;
  double keyboard_scale_angular_yaw_ = 1.0;
  std::chrono::steady_clock::duration motion_suppress_duration_{std::chrono::milliseconds(300)};
  std::chrono::steady_clock::time_point suppress_motion_until_{};
  std::chrono::steady_clock::time_point last_discrete_trigger_time_{};
  bool takeoff_combo_active_ = false;
  bool reset_combo_active_ = false;
  bool mode_next_combo_active_ = false;
  bool emergency_stop_combo_active_ = false;
  bool have_joy_message_ = false;
  sensor_msgs::msg::Joy latest_joy_message_;
  rclcpp::Time last_joy_message_time_{0, 0, RCL_ROS_TIME};
  InputMode input_mode_ = InputMode::kNone;
};

}  // namespace remote_control
