#pragma once
#include <map>
#include <memory>
#include <string>

#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

namespace embedded_rviz {

class Host {
public:
  Host() = default;
  ~Host() = default;

  int      create(const std::string& node_name,
                  const std::string& display_config_path = {});
  void     destroy(int id);
  QWidget* widget(int id) const;
  size_t   size() const { return map_.size(); }

private:
  struct Instance {
    std::string node_name;
    std::string config_path;

    QWidget* root_widget = nullptr;
    rviz_common::RenderPanel* panel = nullptr;
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> ros_node;
    rviz_common::VisualizationManager* manager = nullptr;
  };

  void ensure_rclcpp_init();

  std::map<int, std::unique_ptr<Instance>> map_;
  int next_id_ = 1;
};

} // namespace embedded_rviz
