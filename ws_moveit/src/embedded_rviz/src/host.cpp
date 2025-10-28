#include <embedded_rviz/host.hpp>

#include <QVBoxLayout>
#include <QString>

#include <utility>   // std::move
#include <memory>    // std::make_unique

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
  inst->node_name  = node_name;
  inst->config_path = display_config_path;

  // UI Grundgerüst
  inst->root_widget = new QWidget();
  auto* vlayout = new QVBoxLayout(inst->root_widget);
  vlayout->setContentsMargins(0, 0, 0, 0);

  inst->panel = new rviz_common::RenderPanel(inst->root_widget);

  // Rolling: RosNodeAbstraction direkt konstruieren (ohne Factory-Header)
  rclcpp::NodeOptions opts;
  inst->ros_node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
      node_name.c_str(), opts);

  // VisualizationManager benötigt WeakPtr auf das Iface + Clock
  inst->manager = new rviz_common::VisualizationManager(
      inst->panel,
      rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr(inst->ros_node),
      /* window_manager = */ nullptr,
      inst->ros_node->get_raw_node()->get_clock());

  // DisplayContext an RenderPanel binden
  inst->panel->initialize(inst->manager);

  // Falls du eine .rviz-Konfig laden willst
  if (!inst->config_path.empty()) {
    rviz_common::YamlConfigReader reader;
    rviz_common::Config config;
    reader.readFile(config, QString::fromStdString(inst->config_path));
    inst->manager->load(config);
  }

  inst->manager->initialize();

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
