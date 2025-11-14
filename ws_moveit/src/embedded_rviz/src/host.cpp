#include "embedded_rviz/host.hpp"

#include <memory>
#include <stdexcept>

#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

namespace embedded_rviz {

struct Host::Instance {
  QWidget*                                  root_widget = nullptr;
  rviz_common::RenderPanel*                 panel = nullptr;
  rviz_common::VisualizationManager*        manager = nullptr;
  rviz_common::ros_integration::RosNodeAbstraction::SharedPtr ros_node;
};

Host::~Host() { clear(); }

QWidget* Host::widget(int id) const {
  auto it = instances_.find(id);
  return (it == instances_.end()) ? nullptr : it->second->root_widget;
}

int Host::create(const std::string& node_name, const std::string& config_path) {
  const int id = ++next_id_;
  auto inst = std::make_unique<Instance>();

  inst->root_widget = new QWidget();
  auto* vlayout = new QVBoxLayout(inst->root_widget);
  vlayout->setContentsMargins(0,0,0,0);

  // RenderPanel
  inst->panel = new rviz_common::RenderPanel(inst->root_widget);
  vlayout->addWidget(inst->panel);

  // ROS-Abstraktion (Rolling): SharedPtr + Clock vom raw_node
  inst->ros_node = rviz_common::ros_integration::RosNodeAbstraction::make_shared(node_name);
  auto raw_node = inst->ros_node->get_raw_node();
  auto clock    = raw_node->get_clock();

  // VisualizationManager: (panel, RosNodeAbstractionIface::WeakPtr, WindowManagerInterface*, Clock)
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr weak_ros(inst->ros_node);
  inst->manager = new rviz_common::VisualizationManager(inst->panel, weak_ros, nullptr, clock);

  // Panel initialisieren – Rolling: getDisplayContext() ist die offizielle API
  inst->panel->initialize(inst->manager->getDisplayContext());
  inst->manager->initialize();
  inst->manager->startUpdate();

  // Optional: RViz-Config laden (sicher)
  if (!config_path.empty()) {
    rviz_common::YamlConfigReader reader;
    rviz_common::Config cfg;
    reader.readFile(cfg, QString::fromStdString(config_path));
    if (!reader.error().isEmpty()) {
      // Nur loggen; kein throw, damit es nicht crasht wenn Datei fehlt
      fprintf(stderr, "[embedded_rviz] YamlConfigReader error: %s\n",
              reader.error().toStdString().c_str());
    } else {
      inst->manager->load(cfg);
    }
  }

  instances_.emplace(id, std::move(inst));
  return id;
}

void Host::destroy(int id) {
  auto it = instances_.find(id);
  if (it == instances_.end()) return;

  auto& inst = it->second;
  // Reihenfolge: Manager → Panel → Root (Qt zerstört Kinder)
  if (inst->manager) { delete inst->manager; inst->manager = nullptr; }
  // inst->panel wird als Kind von root_widget mit gelöscht
  if (inst->root_widget) { delete inst->root_widget; inst->root_widget = nullptr; }

  instances_.erase(it);
}

void Host::clear() {
  auto copy = instances_;
  for (auto& kv : copy) { destroy(kv.first); }
}

} // namespace embedded_rviz
