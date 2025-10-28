#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QString>
#include <QMessageBox>

#include <embedded_rviz/host.hpp>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  // Optional: RViz-Config per CLI: panel_demo /path/to/config.rviz
  std::string cfg;
  if (argc > 1) cfg = argv[1];

  embedded_rviz::Host host;
  const int id = host.create("embedded_rviz_panel", cfg);

  QWidget* root = host.widget(id);
  if (!root) {
    QMessageBox::critical(nullptr, "Embedded RViz",
                          "Fehler: RenderPanel konnte nicht erstellt werden.");
    return 1;
  }

  QWidget window;
  window.setWindowTitle("Embedded RViz Panel Demo");
  auto* lay = new QVBoxLayout(&window);
  lay->setContentsMargins(0, 0, 0, 0);
  lay->addWidget(root);
  window.resize(1000, 700);
  window.show();

  // Beim Beenden ordentlich aufräumen
  QObject::connect(&app, &QApplication::aboutToQuit, [&]() {
    host.destroy(id);
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  });

  return app.exec();
}
