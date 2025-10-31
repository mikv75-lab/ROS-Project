#include <pybind11/pybind11.h>
#include <cstdint>
#include <string>
#include <QWidget>
#include "embedded_rviz/host.hpp"

namespace py = pybind11;
using embedded_rviz::Host;

static Host g_host;

PYBIND11_MODULE(embedded_rviz_py, m) {
  m.doc() = "PyBind11 bindings for embedded RViz host";

  m.def("create",
        [](const std::string& node_name, const std::string& config_path) {
          const int id = g_host.create(node_name, config_path);
          QWidget* w = g_host.widget(id);
          unsigned long long wid = 0ULL;
          if (w) {
            wid = static_cast<unsigned long long>(w->winId());
          }
          return py::make_tuple(id, wid);
        },
        py::arg("node_name"), py::arg("config_path") = std::string{}
  );

  m.def("destroy", [](int id) { g_host.destroy(id); }, py::arg("id"));

  // QWidget* als rohen Pointer rausgeben – wird in PyQt via sip.wrapinstance gewrappt
  m.def("qwidget_ptr",
        [](int id) -> unsigned long long {
          QWidget* w = g_host.widget(id);
          return w ? static_cast<unsigned long long>(reinterpret_cast<std::uintptr_t>(w))
                  : 0ULL;
        },
        py::arg("id"));
}
