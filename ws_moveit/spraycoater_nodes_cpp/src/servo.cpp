// servo_bridge.cpp — TopicsLoader + qos_by_id (no command_type; use set_mode/set_frame only)
// - Binds topics via TopicsLoader (publish_topic/subscribe_topic + qos_by_id)
// - Uses only set_mode and set_frame for switching; no command_type publisher exists anymore.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <memory>
#include <string>
#include <exception>

#include "spraycoater_nodes_cpp/topics_loader.hpp"

using std::placeholders::_1;

namespace {
inline std::string to_lower(std::string s){
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
  return s;
}

template <typename T>
inline T clamp(T v, T lo, T hi){ return std::max(lo, std::min(hi, v)); }

struct Frames {
  std::string tcp;
  std::string world;
  std::string tool_mount; // optional
};

Frames load_frames(const std::string &frames_file){
  if (frames_file.empty()) throw std::runtime_error("frames_file parameter is required");
  YAML::Node root = YAML::LoadFile(frames_file);
  if (!root["frames"]) throw std::runtime_error("frames.yaml: missing 'frames' root key");
  auto frames = root["frames"]; // direct keys or nested under namespace

  Frames out;
  // Case A: direct
  if (frames["tcp"]) out.tcp = frames["tcp"].as<std::string>();
  if (frames["world"]) out.world = frames["world"].as<std::string>();
  if (frames["tool_mount"]) out.tool_mount = frames["tool_mount"].as<std::string>();

  // Case B: nested: take first mapping
  if (out.tcp.empty() || out.world.empty()){
    if (!frames.IsMap()) throw std::runtime_error("frames.yaml: 'frames' must be a mapping");
    for (auto it : frames){
      if (it.first.IsScalar() && it.second.IsMap()){
        auto nsmap = it.second;
        if (out.tcp.empty() && nsmap["tcp"]) out.tcp = nsmap["tcp"].as<std::string>();
        if (out.world.empty() && nsmap["world"]) out.world = nsmap["world"].as<std::string>();
        if (out.tool_mount.empty() && nsmap["tool_mount"]) out.tool_mount = nsmap["tool_mount"].as<std::string>();
        break;
      }
    }
  }

  if (out.tcp.empty() || out.world.empty())
    throw std::runtime_error("frames.yaml: could not resolve 'tcp'/'world' frames");

  return out;
}
} // namespace

class ServoBridge : public rclcpp::Node {
public:
  ServoBridge() : rclcpp::Node("servo")
  {
    // --- Frames ---
    const std::string frames_file = declare_parameter<std::string>("frames_file", "");
    frames_ = load_frames(frames_file);

    // --- Servo params ---
    mode_        = to_lower(declare_parameter<std::string>("servo.default_mode",  "cartesian")); // cartesian|joint
    frame_key_   = to_lower(declare_parameter<std::string>("servo.default_frame", "tcp"));       // tcp|world|tool_mount
    lin_mm_s_    = declare_parameter<double>("servo.speed_limits.cart_linear_mm_s",   100.0);
    ang_deg_s_   = declare_parameter<double>("servo.speed_limits.cart_angular_deg_s", 30.0);
    joint_scale_ = declare_parameter<double>("servo.speed_limits.joint_scale",         0.5);

    // Default-Frame korrekt auflösen (inkl. tool_mount mit Fallbacks)
    if (frame_key_ == "world") {
      current_frame_ = frames_.world;
    } else if (frame_key_ == "tool_mount" && !frames_.tool_mount.empty()) {
      current_frame_ = frames_.tool_mount;
    } else {
      // Standard: tcp
      current_frame_ = frames_.tcp;
      frame_key_ = "tcp";
    }

    // --- Topics/QoS via TopicsLoader ---
    const auto topics_file = declare_parameter<std::string>("topics_file", "");
    const auto qos_file    = declare_parameter<std::string>("qos_file",    "");
    if (topics_file.empty() || qos_file.empty())
      throw std::runtime_error("topics_file/qos_file parameter is required");

    topics_ = std::make_shared<topics::TopicsLoader>(topics_file, qos_file);

    // IDs in topics.yaml unter 'servo'
    twist_out_topic_      = topics_->publish_topic  ("servo", "twist_out");
    joint_out_topic_      = topics_->publish_topic  ("servo", "joint_out");

    cart_in_topic_        = topics_->subscribe_topic("servo", "cartesian_mm");
    joint_in_topic_       = topics_->subscribe_topic("servo", "joint_jog");
    set_mode_topic_       = topics_->subscribe_topic("servo", "set_mode");
    set_frame_topic_      = topics_->subscribe_topic("servo", "set_frame");

    // --- Publishers mit qos_by_id ---
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      twist_out_topic_, topics_->qos_by_id("publish", "servo", "twist_out"));

    joint_pub_ = create_publisher<control_msgs::msg::JointJog>(
      joint_out_topic_, topics_->qos_by_id("publish", "servo", "joint_out"));

    // --- Subscribers mit qos_by_id ---
    cart_sub_  = create_subscription<geometry_msgs::msg::TwistStamped>(
      cart_in_topic_, topics_->qos_by_id("subscribe", "servo", "cartesian_mm"),
      std::bind(&ServoBridge::onCartesianMM, this, _1));

    joint_sub_ = create_subscription<control_msgs::msg::JointJog>(
      joint_in_topic_, topics_->qos_by_id("subscribe", "servo", "joint_jog"),
      std::bind(&ServoBridge::onJointJog, this, _1));

    mode_sub_  = create_subscription<std_msgs::msg::String>(
      set_mode_topic_, topics_->qos_by_id("subscribe", "servo", "set_mode"),
      std::bind(&ServoBridge::onSetMode, this, _1));

    frame_sub_ = create_subscription<std_msgs::msg::String>(
      set_frame_topic_, topics_->qos_by_id("subscribe", "servo", "set_frame"),
      std::bind(&ServoBridge::onSetFrame, this, _1));

    RCLCPP_INFO(get_logger(),
      "servo up. mode=%s frame=%s | pub=%s,%s | sub=%s,%s,%s,%s",
      mode_.c_str(), current_frame_.c_str(),
      twist_out_topic_.c_str(), joint_out_topic_.c_str(),
      cart_in_topic_.c_str(), joint_in_topic_.c_str(),
      set_mode_topic_.c_str(), set_frame_topic_.c_str());
  }

private:
  void onCartesianMM(const geometry_msgs::msg::TwistStamped::SharedPtr in)
  {
    if (mode_ != "cartesian") {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring cartesian twist: mode=%s", mode_.c_str());
      return;
    }

    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = now();
    out.header.frame_id = current_frame_;     // tcp | tool_mount | world

    // mm/s -> m/s (clamped)
    const double max_lin = lin_mm_s_ / 1000.0;
    auto lin_x = in->twist.linear.x / 1000.0;
    auto lin_y = in->twist.linear.y / 1000.0;
    auto lin_z = in->twist.linear.z / 1000.0;
    out.twist.linear.x = clamp(lin_x, -max_lin, max_lin);
    out.twist.linear.y = clamp(lin_y, -max_lin, max_lin);
    out.twist.linear.z = clamp(lin_z, -max_lin, max_lin);

    // deg/s -> rad/s (clamped)
    const double max_rad = ang_deg_s_ * M_PI / 180.0;
    auto ang_x = in->twist.angular.x * M_PI / 180.0;
    auto ang_y = in->twist.angular.y * M_PI / 180.0;
    auto ang_z = in->twist.angular.z * M_PI / 180.0;
    out.twist.angular.x = clamp(ang_x, -max_rad, max_rad);
    out.twist.angular.y = clamp(ang_y, -max_rad, max_rad);
    out.twist.angular.z = clamp(ang_z, -max_rad, max_rad);

    twist_pub_->publish(out);
  }

  void onJointJog(const control_msgs::msg::JointJog::SharedPtr in)
  {
    if (mode_ != "joint") {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring joint jog: mode=%s", mode_.c_str());
      return;
    }

    auto msg = *in;                         // copy
    msg.header.stamp = now();
    for (auto &v : msg.velocities) v *= joint_scale_;   // global scale
    joint_pub_->publish(msg);
  }

  void onSetMode(const std_msgs::msg::String::SharedPtr s)
  {
    const auto raw = to_lower(s->data);
    if (raw == "cartesian" || raw == "cart" || raw == "twist") {
      mode_ = "cartesian";
    } else if (raw == "joint" || raw == "joint_jog") {
      mode_ = "joint";
    } else {
      RCLCPP_WARN(get_logger(), "unknown mode '%s' (use 'cartesian'|'joint')", s->data.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "mode := %s", mode_.c_str());
  }

  void onSetFrame(const std_msgs::msg::String::SharedPtr s)
  {
    const auto v = to_lower(s->data);
    if (v == "world") {
      frame_key_ = "world";
      current_frame_ = frames_.world;
    } else if (v == "tool_mount" || v == "tool") {
      if (!frames_.tool_mount.empty()) {
        frame_key_ = "tool_mount";
        current_frame_ = frames_.tool_mount;
      } else {
        frame_key_ = "tcp";
        current_frame_ = frames_.tcp;
        RCLCPP_WARN(get_logger(), "requested frame '%s' but 'tool_mount' missing in frames.yaml -> fallback to tcp (%s)",
                    s->data.c_str(), current_frame_.c_str());
      }
    } else if (v == "tcp") {
      frame_key_ = "tcp";
      current_frame_ = frames_.tcp;
    } else {
      RCLCPP_WARN(get_logger(), "unknown frame '%s' (use 'tcp'|'tool_mount'|'world')", s->data.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "frame := %s (%s)", frame_key_.c_str(), current_frame_.c_str());
  }

  // --- Members ---
  // Frames
  Frames      frames_{};
  std::string current_frame_;
  std::string frame_key_;

  // Mode & limits
  std::string mode_;
  double lin_mm_s_ = 0.0;
  double ang_deg_s_ = 0.0;
  double joint_scale_ = 1.0;

  // Topics loader + resolved names
  std::shared_ptr<topics::TopicsLoader> topics_;
  std::string twist_out_topic_, joint_out_topic_;
  std::string cart_in_topic_, joint_in_topic_, set_mode_topic_, set_frame_topic_;

  // IO
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr       joint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cart_sub_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr      joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            mode_sub_, frame_sub_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoBridge>());
  rclcpp::shutdown();
  return 0;
}
