#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <cstdint>
#include <string>

#include <QWidget>
#include "embedded_rviz/host.hpp"

namespace py = pybind11;

static embedded_rviz::Host g_host;

PYBIND11_MODULE(embedded_rviz_py, m) {
  m.doc() = "PyBind11 bindings for embedded RViz host";

  m.def("create",
        [](const std::string& node_name, const std::string& config_path) {
          const int id = g_host.create(node_name, config_path);
          QWidget* w = g_host.widget(id);
          if (!w) return py::make_tuple(id, 0ULL);
          auto wid = static_cast<unsigned long long>(
              reinterpret_cast<std::uintptr_t>(w->winId()));
          return py::make_tuple(id, wid);
        },
        py::arg("node_name"),
        py::arg("config_path") = std::string{}
  );

  m.def("destroy", [](int id) { g_host.destroy(id); }, py::arg("id"));

  m.def("wid",
        [](int id) {
          QWidget* w = g_host.widget(id);
          if (!w) return 0ULL;
          auto wid = static_cast<unsigned long long>(
              reinterpret_cast<std::uintptr_t>(w->winId()));
          return wid;
        },
        py::arg("id")
  );

  m.def("size", []() { return static_cast<unsigned long long>(g_host.size()); });
}
