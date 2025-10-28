#pragma once

#include <map>
#include <memory>
#include <string>

#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>  // <- wichtig: NICHT die factory

namespace embedded_rviz {

class Host {
public:
  Host() = default;
  ~Host() = default;

  int      create(const std::string& node_name, const std::string& display_config_path);
  void     destroy(int id);
  QWidget* widget(int id) const;
  size_t   size() const { return map_.size(); }

private:
  static void ensure_rclcpp_init();

  struct Instance {
    QWidget*                                                   root_widget = nullptr;
    rviz_common::RenderPanel*                                  panel = nullptr;
    rviz_common::VisualizationManager*                         manager = nullptr;

    // Rolling: nutze das *Interface* als Handle
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> ros_node;

    std::string                                                node_name;
    std::string                                                config_path;
  };

  int                                                next_id_ = 1;
  std::map<int, std::unique_ptr<Instance>>          map_;
};

} // namespace embedded_rviz
