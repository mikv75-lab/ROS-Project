#include "embedded_rviz/host.hpp"

#include <QVBoxLayout>
#include <QString>

#include <utility>
#include <memory>

namespace embedded_rviz {

void Host::ensure_rclcpp_init() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

int Host::create(const std::string& node_name, const std::string& display_config_path) {
  ensure_rclcpp_init();

  auto inst = std::make_unique<Instance>();
  inst->node_name   = node_name;
  inst->config_path = display_config_path;

  // Root Qt widget
  inst->root_widget = new QWidget();
  auto* vlayout = new QVBoxLayout(inst->root_widget);
  vlayout->setContentsMargins(0, 0, 0, 0);

  // RenderPanel
  inst->panel = new rviz_common::RenderPanel(inst->root_widget);

  // ROS Node Abstraction
  rclcpp::NodeOptions opts;
  inst->ros_node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
      node_name.c_str(), opts);

  // VisualizationManager
  auto clock = inst->ros_node->get_raw_node()->get_clock();
  inst->manager = new rviz_common::VisualizationManager(
      inst->panel,
      rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr(inst->ros_node),
      /* window_manager = */ nullptr,
      clock
  );

  // Initialisieren: Panel bekommt den DisplayContext -> hier: der Manager selbst
  inst->panel->initialize(inst->manager /* DisplayContext* */, /*use_main_scene=*/false);

  // Manager initialisieren & ggf. Config laden
  inst->manager->initialize();

  if (!inst->config_path.empty()) {
    rviz_common::YamlConfigReader reader;
    rviz_common::Config config;
    reader.readFile(config, QString::fromStdString(inst->config_path));
    inst->manager->load(config);
  }

  // Update starten und Panel einhängen
  inst->manager->startUpdate();
  vlayout->addWidget(inst->panel);

  const int id = next_id_++;
  map_.emplace(id, std::move(inst));
  return id;
}

void Host::destroy(int id) {
  auto it = map_.find(id);
  if (it == map_.end()) return;

  auto& inst = it->second;

  if (inst->manager) {
    inst->manager->stopUpdate();
    delete inst->manager;
    inst->manager = nullptr;
  }
  if (inst->panel) {
    delete inst->panel;
    inst->panel = nullptr;
  }
  if (inst->root_widget) {
    delete inst->root_widget;
    inst->root_widget = nullptr;
  }

  map_.erase(it);
}

QWidget* Host::widget(int id) const {
  auto it = map_.find(id);
  if (it == map_.end()) return nullptr;
  return it->second->root_widget;
}

} // namespace embedded_rviz
