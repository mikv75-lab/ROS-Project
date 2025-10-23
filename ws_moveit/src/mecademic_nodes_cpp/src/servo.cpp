// servo_bridge.cpp
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
#include <chrono>

#include "mecademic_nodes_cpp/topics_loader.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace {
inline std::string to_lower(std::string s){
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c){ return std::tolower(c); });
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
  auto frames = root["frames"];

  Frames out;
  if (frames["tcp"])        out.tcp        = frames["tcp"].as<std::string>();
  if (frames["world"])      out.world      = frames["world"].as<std::string>();
  if (frames["tool_mount"]) out.tool_mount = frames["tool_mount"].as<std::string>();

  if (out.tcp.empty() || out.world.empty()){
    if (!frames.IsMap()) throw std::runtime_error("frames.yaml: 'frames' must be a mapping");
    for (auto it : frames){
      if (it.first.IsScalar() && it.second.IsMap()){
        auto nsmap = it.second;
        if (out.tcp.empty()        && nsmap["tcp"])        out.tcp        = nsmap["tcp"].as<std::string>();
        if (out.world.empty()      && nsmap["world"])      out.world      = nsmap["world"].as<std::string>();
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
  ServoBridge() : rclcpp::Node("servo"), steady_(rclcpp::ClockType::RCL_STEADY_TIME)
  {
    // --- Frames ---
    const std::string frames_file = declare_parameter<std::string>("frames_file", "");
    frames_ = load_frames(frames_file);

    // --- Servo params ---
    mode_        = to_lower(declare_parameter<std::string>("servo.default_mode",  "cartesian")); // cartesian|joint
    frame_key_   = to_lower(declare_parameter<std::string>("servo.default_frame", "tcp"));       // tcp|world
    lin_mm_s_    = declare_parameter<double>("servo.speed_limits.cart_linear_mm_s",   100.0);
    ang_deg_s_   = declare_parameter<double>("servo.speed_limits.cart_angular_deg_s", 30.0);
    joint_scale_ = declare_parameter<double>("servo.speed_limits.joint_scale",         0.5);
    current_frame_ = (frame_key_ == "world") ? frames_.world : frames_.tcp;

    // --- Topics/QoS via TopicsLoader ---
    const auto topics_file = declare_parameter<std::string>("topics_file", "");
    const auto qos_file    = declare_parameter<std::string>("qos_file",    "");
    if (topics_file.empty() || qos_file.empty())
      throw std::runtime_error("topics_file/qos_file parameter is required");
    topics_ = std::make_shared<topics::TopicsLoader>(topics_file, qos_file);

    // Resolve names
    twist_out_topic_ = topics_->publish_topic  ("servo", "twist_out");
    joint_out_topic_ = topics_->publish_topic  ("servo", "joint_out");
    cart_in_topic_   = topics_->subscribe_topic("servo", "cartesian_mm");
    joint_in_topic_  = topics_->subscribe_topic("servo", "joint_jog");
    set_mode_topic_  = topics_->subscribe_topic("servo", "set_mode");
    set_frame_topic_ = topics_->subscribe_topic("servo", "set_frame");

    // --- Publishers
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        twist_out_topic_, topics_->qos_by_id("publish", "servo", "twist_out"));
    joint_pub_ = create_publisher<control_msgs::msg::JointJog>(
        joint_out_topic_, topics_->qos_by_id("publish", "servo", "joint_out"));

    // --- Subscribers
    cart_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        cart_in_topic_, topics_->qos_by_id("subscribe", "servo", "cartesian_mm"),
        std::bind(&ServoBridge::onCartesianMM, this, _1));
    joint_sub_ = create_subscription<control_msgs::msg::JointJog>(
        joint_in_topic_, topics_->qos_by_id("subscribe", "servo", "joint_jog"),
        std::bind(&ServoBridge::onJointJog, this, _1));
    mode_sub_ = create_subscription<std_msgs::msg::String>(
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

    // --- Optional: Self-Test beim Start
    const bool do_test = declare_parameter<bool>("self_test.enable", false);
    if (do_test) setup_self_test();
  }

private:
  // ===== Callbacks (Bridge) =====
  void onCartesianMM(const geometry_msgs::msg::TwistStamped::SharedPtr in)
  {
    if (mode_ != "cartesian") return;

    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = steady_.now();         // monotone!
    out.header.frame_id = current_frame_;     // tcp/world

    const double max_lin = lin_mm_s_ / 1000.0;                      // mm/s -> m/s
    out.twist.linear.x = clamp(in->twist.linear.x / 1000.0, -max_lin, max_lin);
    out.twist.linear.y = clamp(in->twist.linear.y / 1000.0, -max_lin, max_lin);
    out.twist.linear.z = clamp(in->twist.linear.z / 1000.0, -max_lin, max_lin);

    const double max_rad = ang_deg_s_ * M_PI / 180.0;               // deg/s -> rad/s
    out.twist.angular.x = clamp(in->twist.angular.x * M_PI/180.0, -max_rad, max_rad);
    out.twist.angular.y = clamp(in->twist.angular.y * M_PI/180.0, -max_rad, max_rad);
    out.twist.angular.z = clamp(in->twist.angular.z * M_PI/180.0, -max_rad, max_rad);

    twist_pub_->publish(out);
  }

  void onJointJog(const control_msgs::msg::JointJog::SharedPtr in)
  {
    if (mode_ != "joint") return;

    auto msg = *in;                      // copy
    msg.header.stamp = steady_.now();    // monotone!
    for (auto &v : msg.velocities) v *= joint_scale_;
    joint_pub_->publish(msg);
  }

  void onSetMode(const std_msgs::msg::String::SharedPtr s)
  {
    const auto v = to_lower(s->data);
    if (v == "cartesian" || v == "cart" || v == "twist")      mode_ = "cartesian";
    else if (v == "joint" || v == "joint_jog")                mode_ = "joint";
    else { RCLCPP_WARN(get_logger(), "unknown mode '%s'", s->data.c_str()); return; }
    RCLCPP_INFO(get_logger(), "mode := %s", mode_.c_str());
  }

  void onSetFrame(const std_msgs::msg::String::SharedPtr s)
  {
    const auto v = to_lower(s->data);
    if      (v == "tcp")   { frame_key_ = v; current_frame_ = frames_.tcp; }
    else if (v == "world") { frame_key_ = v; current_frame_ = frames_.world; }
    else { RCLCPP_WARN(get_logger(), "unknown frame '%s'", s->data.c_str()); return; }
    RCLCPP_INFO(get_logger(), "frame := %s (%s)", frame_key_.c_str(), current_frame_.c_str());
  }

  // ===== Self-Test =====
  void setup_self_test()
  {
    // Parameter
    const int  delay_ms    = declare_parameter<int>("self_test.delay_ms", 300);
    cart_lin_mm_ = declare_parameter<double>("self_test.cart_lin_x_mm_s", 50.0);
    cart_ang_deg_= declare_parameter<double>("self_test.cart_ang_z_deg_s", 10.0);
    joint_name_  = declare_parameter<std::string>("self_test.joint_name", "meca_axis_1_joint");
    joint_vel_   = declare_parameter<double>("self_test.joint_vel", 0.3);
    const std::string frame_param = to_lower(declare_parameter<std::string>("self_test.frame", "world"));

    // temporäre Pubs (auf Eingangs-Topics) & Echos (auf Ausgangs-Topics)
    t_set_mode_pub_  = create_publisher<std_msgs::msg::String>(set_mode_topic_,
                        topics_->qos_by_id("subscribe","servo","set_mode"));
    t_set_frame_pub_ = create_publisher<std_msgs::msg::String>(set_frame_topic_,
                        topics_->qos_by_id("subscribe","servo","set_frame"));
    t_twist_in_pub_  = create_publisher<geometry_msgs::msg::TwistStamped>(cart_in_topic_,
                        topics_->qos_by_id("subscribe","servo","cartesian_mm"));
    t_joint_in_pub_  = create_publisher<control_msgs::msg::JointJog>(joint_in_topic_,
                        topics_->qos_by_id("subscribe","servo","joint_jog"));

    t_twist_echo_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        twist_out_topic_, topics_->qos_by_id("publish","servo","twist_out"),
        [this](const geometry_msgs::msg::TwistStamped &m){
          RCLCPP_INFO(this->get_logger(),
            "[self-test] got delta_twist_cmds: lin=(%.4f,%.4f,%.4f) m/s  ang=(%.4f,%.4f,%.4f) rad/s  frame=%s",
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z,
            m.twist.angular.x, m.twist.angular.y, m.twist.angular.z,
            m.header.frame_id.c_str());
          twist_echoed_ = true;
        });

    t_joint_echo_sub_ = create_subscription<control_msgs::msg::JointJog>(
        joint_out_topic_, topics_->qos_by_id("publish","servo","joint_out"),
        [this](const control_msgs::msg::JointJog &m){
          if(!m.joint_names.empty() && !m.velocities.empty()){
            RCLCPP_INFO(this->get_logger(),
              "[self-test] got delta_joint_cmds: %s := %.4f",
              m.joint_names.front().c_str(), m.velocities.front());
          } else {
            RCLCPP_INFO(this->get_logger(), "[self-test] got delta_joint_cmds (empty)");
          }
          joint_echoed_ = true;
        });

    // Schritt 1: Frame & Cartesian
    timer1_ = create_wall_timer(std::chrono::milliseconds(delay_ms), [this, frame_param](){
      std_msgs::msg::String f;  f.data = frame_param;
      std_msgs::msg::String m;  m.data = "cartesian";
      t_set_frame_pub_->publish(f);
      t_set_mode_pub_->publish(m);

      geometry_msgs::msg::TwistStamped in;
      in.header.frame_id = (frame_param=="world") ? frames_.world : frames_.tcp;
      in.twist.linear.x  = cart_lin_mm_;
      in.twist.angular.z = cart_ang_deg_;
      t_twist_in_pub_->publish(in);
      RCLCPP_INFO(this->get_logger(), "[self-test] sent 1x cartesian_mm (%.1f mm/s, %.1f deg/s)",
                  cart_lin_mm_, cart_ang_deg_);

      // Schritt 2 nach 1s: Joint
      timer2_ = create_wall_timer(1000ms, [this](){
        std_msgs::msg::String m;  m.data = "joint";
        t_set_mode_pub_->publish(m);

        control_msgs::msg::JointJog jj;
        jj.joint_names = {joint_name_};
        jj.velocities  = {joint_vel_};
        t_joint_in_pub_->publish(jj);
        RCLCPP_INFO(this->get_logger(), "[self-test] sent 1x joint_jog (%s := %.3f)",
                    joint_name_.c_str(), joint_vel_);
      });

      // Timer 1 nur einmal
      timer1_->cancel();
    });
  }

  // --- Members ---
  rclcpp::Clock steady_;
  // Frames & mode
  Frames      frames_{};
  std::string current_frame_;
  std::string frame_key_;
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
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr      joint_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cart_sub_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr      joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            mode_sub_, frame_sub_;

  // Self-Test helpers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                t_set_mode_pub_, t_set_frame_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr     t_twist_in_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr          t_joint_in_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr  t_twist_echo_sub_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr       t_joint_echo_sub_;
  rclcpp::TimerBase::SharedPtr timer1_, timer2_;
  bool twist_echoed_{false}, joint_echoed_{false};
  // self-test params
  double cart_lin_mm_{50.0}, cart_ang_deg_{10.0};
  std::string joint_name_{"meca_axis_1_joint"};
  double joint_vel_{0.3};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoBridge>());
  rclcpp::shutdown();
  return 0;
}
