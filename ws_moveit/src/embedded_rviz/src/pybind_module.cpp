#include <pybind11/pybind11.h>
#include <string>
#include <cstdint>
#include "embedded_rviz/host.hpp"

namespace py = pybind11;
using embedded_rviz::Host;

static Host g_host;

PYBIND11_MODULE(embedded_rviz, m) {
  m.def("create",  [](const std::string& node_name, const std::string& config_path){
      return g_host.create(node_name, config_path);
    }, py::arg("node_name"), py::arg("config_path") = "");

  m.def("destroy", [](int id){ g_host.destroy(id); }, py::arg("id"));

  m.def("wid",     [](int id){
      QWidget* w = g_host.widget(id);
      return static_cast<unsigned long long>(reinterpret_cast<std::uintptr_t>(w));
    }, py::arg("id"));

  m.def("size",    [](){ return g_host.size(); });
}
