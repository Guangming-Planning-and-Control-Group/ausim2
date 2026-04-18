#include "remote_control_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <termios.h>

#include <sys/select.h>
#include <unistd.h>

namespace remote_control {
namespace {

constexpr char kDefaultJoyTopic[] = "/joy";
constexpr char kDefaultCmdVelTopic[] = "/uav1/cmd_vel";
constexpr char kDefaultTeleopEventTopic[] = "/uav1/teleop/event";
constexpr char kDefaultTakeoffService[] = "/uav1/takeoff";
constexpr char kDefaultResetService[] = "/uav1/sim/reset";

double ApplyDeadzone(double value, double deadzone) {
  if (std::abs(value) < deadzone) {
    return 0.0;
  }
  return value;
}

char NormalizeKey(char key) { return static_cast<char>(std::tolower(static_cast<unsigned char>(key))); }

}  // namespace

TerminalKeyboard::TerminalKeyboard(bool enabled, double key_timeout_seconds)
    : key_timeout_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(key_timeout_seconds))),
      original_termios_(new termios{}) {
  if (!enabled) {
    return;
  }
  if (!isatty(STDIN_FILENO)) {
    return;
  }
  if (tcgetattr(STDIN_FILENO, original_termios_) != 0) {
    return;
  }

  termios raw = *original_termios_;
  raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
    return;
  }

  had_termios_ = true;
  terminal_configured_ = true;
  available_.store(true);
  running_.store(true);
  reader_thread_ = std::thread(&TerminalKeyboard::ReaderLoop, this);
}

TerminalKeyboard::~TerminalKeyboard() {
  running_.store(false);
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
  if (terminal_configured_ && had_termios_ && original_termios_ != nullptr) {
    tcsetattr(STDIN_FILENO, TCSANOW, original_termios_);
  }
  delete original_termios_;
}

void TerminalKeyboard::RegisterEventKey(char key, std::string event_name) {
  if (key == '\0') {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  event_keys_[NormalizeKey(key)] = std::move(event_name);
}

void TerminalKeyboard::ClearEventKeys() {
  std::lock_guard<std::mutex> lock(mutex_);
  event_keys_.clear();
}

geometry_msgs::msg::Twist TerminalKeyboard::BuildTwist(double linear_x_scale, double linear_y_scale, double linear_z_scale,
                                                       double angular_yaw_scale) const {
  geometry_msgs::msg::Twist command;
  const TimePoint now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);

  const double forward = key_state_.forward_until > now ? 1.0 : 0.0;
  const double backward = key_state_.backward_until > now ? 1.0 : 0.0;
  const double left = key_state_.left_until > now ? 1.0 : 0.0;
  const double right = key_state_.right_until > now ? 1.0 : 0.0;
  const double up = key_state_.up_until > now ? 1.0 : 0.0;
  const double down = key_state_.down_until > now ? 1.0 : 0.0;
  const double yaw_left = key_state_.yaw_left_until > now ? 1.0 : 0.0;
  const double yaw_right = key_state_.yaw_right_until > now ? 1.0 : 0.0;

  command.linear.x = (forward - backward) * linear_x_scale;
  command.linear.y = (left - right) * linear_y_scale;
  command.linear.z = (up - down) * linear_z_scale;
  command.angular.z = (yaw_left - yaw_right) * angular_yaw_scale;
  return command;
}

std::optional<std::string> TerminalKeyboard::ConsumeEvent() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_events_.empty()) {
    return std::nullopt;
  }
  std::string event_name = std::move(pending_events_.front());
  pending_events_.pop_front();
  return event_name;
}

void TerminalKeyboard::ReaderLoop() {
  while (running_.load()) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(STDIN_FILENO, &read_fds);

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
      continue;
    }

    char value = '\0';
    const ssize_t bytes = read(STDIN_FILENO, &value, 1);
    if (bytes == 1) {
      HandleChar(value);
    }
  }
}

void TerminalKeyboard::HandleChar(char value) {
  const TimePoint until = std::chrono::steady_clock::now() + key_timeout_;
  std::lock_guard<std::mutex> lock(mutex_);
  switch (NormalizeKey(value)) {
    case 'w':
      key_state_.forward_until = until;
      return;
    case 's':
      key_state_.backward_until = until;
      return;
    case 'a':
      key_state_.left_until = until;
      return;
    case 'd':
      key_state_.right_until = until;
      return;
    case 'r':
      key_state_.up_until = until;
      return;
    case 'f':
      key_state_.down_until = until;
      return;
    case 'j':
      key_state_.yaw_left_until = until;
      return;
    case 'l':
      key_state_.yaw_right_until = until;
      return;
    case ' ':
      ClearMovementLocked();
      return;
    default:
      break;
  }

  // Non-movement keys: look up the user-configured event map.
  const auto it = event_keys_.find(NormalizeKey(value));
  if (it != event_keys_.end()) {
    pending_events_.push_back(it->second);
  }
}

void TerminalKeyboard::ClearMovementLocked() {
  const TimePoint zero{};
  key_state_.forward_until = zero;
  key_state_.backward_until = zero;
  key_state_.left_until = zero;
  key_state_.right_until = zero;
  key_state_.up_until = zero;
  key_state_.down_until = zero;
  key_state_.yaw_left_until = zero;
  key_state_.yaw_right_until = zero;
}

RemoteControlNode::RemoteControlNode() : Node("remote_control_node") {
  LoadParameters();
  LoadEventBindings();

  cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  if (!teleop_event_topic_.empty()) {
    teleop_event_publisher_ = create_publisher<std_msgs::msg::String>(teleop_event_topic_, 10);
  }
  joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SensorDataQoS(), std::bind(&RemoteControlNode::OnJoy, this, std::placeholders::_1));

  if (!takeoff_service_.empty()) {
    takeoff_client_ = create_client<std_srvs::srv::Trigger>(takeoff_service_);
  }
  if (!reset_service_.empty()) {
    reset_client_ = create_client<std_srvs::srv::Trigger>(reset_service_);
  }

  keyboard_ = std::make_unique<TerminalKeyboard>(keyboard_enabled_, keyboard_key_timeout_seconds_);
  if (keyboard_enabled_ && !keyboard_->available()) {
    RCLCPP_WARN(get_logger(), "keyboard fallback requested but stdin is not a TTY; keyboard control is disabled");
  }
  for (const auto& [event_name, binding] : event_bindings_) {
    if (binding.keyboard_key != '\0') {
      keyboard_->RegisterEventKey(binding.keyboard_key, event_name);
    }
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz_));
  publish_timer_ = create_wall_timer(period, std::bind(&RemoteControlNode::OnPublishTimer, this));
}

void RemoteControlNode::LoadParameters() {
  joy_topic_ = declare_parameter<std::string>("topics.joy", kDefaultJoyTopic);
  cmd_vel_topic_ = declare_parameter<std::string>("topics.cmd_vel", kDefaultCmdVelTopic);
  teleop_event_topic_ = declare_parameter<std::string>("topics.teleop_event", kDefaultTeleopEventTopic);
  takeoff_service_ = declare_parameter<std::string>("services.takeoff", kDefaultTakeoffService);
  reset_service_ = declare_parameter<std::string>("services.reset", kDefaultResetService);

  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
  joy_timeout_seconds_ = declare_parameter<double>("joy_timeout", 0.5);
  deadzone_ = declare_parameter<double>("deadzone", 0.15);
  require_enable_button_ = declare_parameter<bool>("require_enable_button", false);
  enable_button_ = declare_parameter<int>("enable_button", 4);
  axis_linear_x_ = declare_parameter<int>("axes.linear.x", 4);
  axis_linear_y_ = declare_parameter<int>("axes.linear.y", 0);
  axis_linear_z_ = declare_parameter<int>("axes.linear.z", 1);
  axis_angular_yaw_ = declare_parameter<int>("axes.angular.yaw", 3);
  scale_linear_x_ = declare_parameter<double>("scale.linear.x", 0.6);
  scale_linear_y_ = declare_parameter<double>("scale.linear.y", 0.6);
  scale_linear_z_ = declare_parameter<double>("scale.linear.z", 0.5);
  scale_angular_yaw_ = declare_parameter<double>("scale.angular.yaw", 1.0);

  command_cooldown_seconds_ = declare_parameter<double>("command_cooldown", 0.5);
  keyboard_enabled_ = declare_parameter<bool>("keyboard.enabled", true);
  keyboard_key_timeout_seconds_ = declare_parameter<double>("keyboard.key_timeout", 0.25);
  keyboard_scale_linear_x_ = declare_parameter<double>("keyboard.scale.linear.x", 0.6);
  keyboard_scale_linear_y_ = declare_parameter<double>("keyboard.scale.linear.y", 0.6);
  keyboard_scale_linear_z_ = declare_parameter<double>("keyboard.scale.linear.z", 0.5);
  keyboard_scale_angular_yaw_ = declare_parameter<double>("keyboard.scale.angular.yaw", 1.0);
  motion_suppress_duration_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(declare_parameter<double>("motion_suppress_after_command", 0.3)));

  if (publish_rate_hz_ <= 0.0) {
    throw std::runtime_error("publish_rate_hz must be positive");
  }
  if (joy_timeout_seconds_ < 0.0) {
    throw std::runtime_error("joy_timeout must be non-negative");
  }
}

void RemoteControlNode::LoadEventBindings() {
  // Preferred style: events.<name>.buttons  + events.<name>.keyboard
  // Legacy style:    buttons.<takeoff|reset|mode_next|estop>
  // Legacy keyboard keys (t/x/m/q) are retained as fallback defaults.
  struct LegacyDefault {
    const char* name;
    std::vector<std::int64_t> buttons;
    char keyboard_key;
  };
  const LegacyDefault legacy_defaults[] = {
      {"takeoff", {4, 0}, 't'},
      {"land", {5, 0}, 'g'},
      {"reset", {6, 7}, 'x'},
      {"mode_next", {-1}, 'm'},
      {"estop", {-1}, 'q'},
  };

  for (const LegacyDefault& def : legacy_defaults) {
    const std::string name(def.name);
    const std::string buttons_param = "events." + name + ".buttons";
    const std::string keyboard_param = "events." + name + ".keyboard";
    std::vector<std::int64_t> buttons = declare_parameter<std::vector<std::int64_t>>(buttons_param, def.buttons);
    std::string keyboard_str = declare_parameter<std::string>(keyboard_param, std::string(1, def.keyboard_key));

    // Legacy fallback for buttons: if the caller set the old `buttons.<name>`
    // param but didn't yet populate the new style, honour it.
    const std::string legacy_buttons_param = "buttons." + name;
    std::vector<std::int64_t> legacy_buttons = declare_parameter<std::vector<std::int64_t>>(legacy_buttons_param, def.buttons);
    const bool preferred_uses_default = buttons == def.buttons;
    const bool legacy_overridden = legacy_buttons != def.buttons;
    if (preferred_uses_default && legacy_overridden) {
      buttons = std::move(legacy_buttons);
    }

    EventBinding binding;
    binding.buttons = ToIntVector(buttons);
    // Filter out -1 sentinels used historically to disable a combo.
    binding.buttons.erase(std::remove(binding.buttons.begin(), binding.buttons.end(), -1), binding.buttons.end());
    binding.keyboard_key = keyboard_str.empty() ? '\0' : keyboard_str.front();
    event_bindings_.emplace(name, std::move(binding));
    event_combo_active_.emplace(name, false);
  }

  // Allow arbitrary user-defined events via `events.<name>.buttons/keyboard`.
  // (Beyond the legacy set, we can't enumerate without a parameter namespace
  // walker; future extension can scan `list_parameters`.)
}

void RemoteControlNode::OnJoy(const sensor_msgs::msg::Joy::SharedPtr message) {
  if (!have_joy_message_) {
    RCLCPP_INFO(get_logger(), "received first joy message on %s", joy_topic_.c_str());
  }
  latest_joy_message_ = *message;
  have_joy_message_ = true;
  last_joy_message_time_ = now();

  for (auto& [event_name, binding] : event_bindings_) {
    const bool pressed = !binding.buttons.empty() && ButtonsPressed(*message, binding.buttons);
    bool& was_active = event_combo_active_[event_name];
    if (pressed && !was_active) {
      TriggerAction(event_name, "joystick");
    }
    was_active = pressed;
  }
}

void RemoteControlNode::OnPublishTimer() {
  while (keyboard_ != nullptr) {
    std::optional<std::string> event = keyboard_->ConsumeEvent();
    if (!event.has_value()) {
      break;
    }
    TriggerAction(*event, "keyboard");
  }

  geometry_msgs::msg::Twist command;
  const bool joystick_active = have_joy_message_ && (now() - last_joy_message_time_).seconds() <= joy_timeout_seconds_;
  if (!joystick_active) {
    for (auto& [name, active] : event_combo_active_) {
      (void)name;
      active = false;
    }
  }

  if (std::chrono::steady_clock::now() < suppress_motion_until_) {
    command = geometry_msgs::msg::Twist();
    UpdateInputMode(joystick_active ? InputMode::kJoystick : (keyboard_ != nullptr && keyboard_->available() ? InputMode::kKeyboard : InputMode::kNone));
  } else if (joystick_active) {
    command = BuildJoyTwist(latest_joy_message_);
    UpdateInputMode(InputMode::kJoystick);
  } else if (keyboard_ != nullptr && keyboard_->available()) {
    command = keyboard_->BuildTwist(keyboard_scale_linear_x_, keyboard_scale_linear_y_, keyboard_scale_linear_z_, keyboard_scale_angular_yaw_);
    UpdateInputMode(InputMode::kKeyboard);
  } else {
    UpdateInputMode(InputMode::kNone);
  }

  cmd_vel_publisher_->publish(command);
}

void RemoteControlNode::TriggerAction(const std::string& event_name, const char* source) {
  if (event_name.empty()) {
    return;
  }

  const auto now_steady = std::chrono::steady_clock::now();
  if (last_discrete_trigger_time_ != std::chrono::steady_clock::time_point{} &&
      std::chrono::duration<double>(now_steady - last_discrete_trigger_time_).count() < command_cooldown_seconds_) {
    return;
  }

  last_discrete_trigger_time_ = now_steady;
  suppress_motion_until_ = now_steady + motion_suppress_duration_;
  PublishZeroCommand();

  RCLCPP_INFO(get_logger(), "triggering %s from %s", event_name.c_str(), source);
  if (teleop_event_publisher_ != nullptr) {
    std_msgs::msg::String message;
    message.data = event_name;
    teleop_event_publisher_->publish(message);
    return;
  }

  // Service fallback for the two events that have dedicated std_srvs::Trigger
  // endpoints on the sim side. Other events require the teleop_event topic.
  if (event_name == "takeoff") {
    CallTriggerService("takeoff", takeoff_client_);
    return;
  }
  if (event_name == "reset") {
    CallTriggerService("reset", reset_client_);
    return;
  }
  RCLCPP_WARN(get_logger(), "teleop event publisher is not configured for '%s'", event_name.c_str());
}

void RemoteControlNode::CallTriggerService(const std::string& label, const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client) {
  if (client == nullptr) {
    RCLCPP_WARN(get_logger(), "%s service client is not configured", label.c_str());
    return;
  }
  if (!client->wait_for_service(std::chrono::milliseconds(0))) {
    RCLCPP_WARN(get_logger(), "%s service is not available", label.c_str());
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
      request, [this, label](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (response->success) {
            RCLCPP_INFO(get_logger(), "%s service succeeded: %s", label.c_str(), response->message.c_str());
          } else {
            RCLCPP_WARN(get_logger(), "%s service rejected: %s", label.c_str(), response->message.c_str());
          }
        } catch (const std::exception& error) {
          RCLCPP_ERROR(get_logger(), "%s service call failed: %s", label.c_str(), error.what());
        }
      });
}

void RemoteControlNode::PublishZeroCommand() { cmd_vel_publisher_->publish(geometry_msgs::msg::Twist()); }

void RemoteControlNode::UpdateInputMode(InputMode mode) {
  if (mode == input_mode_) {
    return;
  }

  input_mode_ = mode;
  switch (input_mode_) {
    case InputMode::kJoystick:
      RCLCPP_INFO(get_logger(), "using joystick control");
      break;
    case InputMode::kKeyboard:
      RCLCPP_INFO(get_logger(),
                  "no fresh joystick input on %s, switching to keyboard fallback; ensure joy_node is running and autorepeat_rate is enabled",
                  joy_topic_.c_str());
      break;
    case InputMode::kNone:
      RCLCPP_INFO(get_logger(),
                  "no fresh joystick input on %s and no keyboard TTY, publishing zero cmd_vel; ensure joy_node is running and autorepeat_rate is enabled",
                  joy_topic_.c_str());
      break;
  }
}

geometry_msgs::msg::Twist RemoteControlNode::BuildJoyTwist(const sensor_msgs::msg::Joy& message) const {
  geometry_msgs::msg::Twist command;
  if (require_enable_button_ && !ButtonPressed(message, enable_button_)) {
    return command;
  }

  command.linear.x = AxisValue(message, axis_linear_x_, scale_linear_x_);
  command.linear.y = AxisValue(message, axis_linear_y_, scale_linear_y_);
  command.linear.z = AxisValue(message, axis_linear_z_, scale_linear_z_);
  command.angular.z = AxisValue(message, axis_angular_yaw_, scale_angular_yaw_);
  return command;
}

bool RemoteControlNode::ButtonPressed(const sensor_msgs::msg::Joy& message, int button_index) const {
  return button_index >= 0 && button_index < static_cast<int>(message.buttons.size()) && message.buttons[button_index] != 0;
}

bool RemoteControlNode::ButtonsPressed(const sensor_msgs::msg::Joy& message, const std::vector<int>& button_indices) const {
  if (button_indices.empty()) {
    return false;
  }
  return std::all_of(button_indices.begin(), button_indices.end(), [this, &message](int button_index) {
    return ButtonPressed(message, button_index);
  });
}

double RemoteControlNode::AxisValue(const sensor_msgs::msg::Joy& message, int axis_index, double scale) const {
  if (axis_index < 0 || axis_index >= static_cast<int>(message.axes.size())) {
    return 0.0;
  }
  return ApplyDeadzone(message.axes[axis_index], deadzone_) * scale;
}

std::vector<int> RemoteControlNode::ToIntVector(const std::vector<std::int64_t>& values) {
  std::vector<int> result;
  result.reserve(values.size());
  for (const std::int64_t value : values) {
    result.push_back(static_cast<int>(value));
  }
  return result;
}

}  // namespace remote_control
