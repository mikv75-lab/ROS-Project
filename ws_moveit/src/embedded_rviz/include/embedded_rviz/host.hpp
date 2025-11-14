#pragma once
#include <map>
#include <memory>
#include <string>

#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

namespace embedded_rviz {

class Host {
public:
  struct Instance {
    QWidget* root_widget{nullptr};
    rviz_common::RenderPanel* panel{nullptr};
    rviz_common::VisualizationManager* manager{nullptr};

    // Rolling: roher rclcpp-Node + RosNodeAbstraction
    rclcpp::Node::SharedPtr raw_node;
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> ros_node;
  };

  int create(const std::string& node_name, const std::string& config_path);
  void destroy(int id);
  QWidget* widget(int id) const;

private:
  int next_id_{1};
  std::map<int, std::unique_ptr<Instance>> map_;
};

}  // namespace embedded_rviz
