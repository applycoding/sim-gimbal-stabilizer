// SPDX-License-Identifier: Apache-2.0
#include <pybind11/pybind11.h>

#include "sgs/sim/scenario.hpp"
#include "sgs/types.hpp"

namespace py = pybind11;

PYBIND11_MODULE(sgs_core, m) {
  m.doc() = "sim-gimbal-stabilizer Python bindings (simulation only).";

  py::class_<sgs::PidGains>(m, "PidGains")
      .def(py::init<>())
      .def_readwrite("kp", &sgs::PidGains::kp)
      .def_readwrite("ki", &sgs::PidGains::ki)
      .def_readwrite("kd", &sgs::PidGains::kd);

  py::class_<sgs::Gains>(m, "Gains")
      .def(py::init<>())
      .def_readwrite("rate", &sgs::Gains::rate)
      .def_readwrite("boresight", &sgs::Gains::boresight);

  m.def("default_gains", &sgs::default_gains);
  m.def("default_limits", &sgs::default_limits);

  m.def(
      "run_step_pitch_max_error",
      [](uint32_t seed, double duration_s) {
        sgs::sim::Scenario sc{};
        sc.kind = sgs::sim::ScenarioKind::StepPitch;
        sc.duration_s = duration_s;
        sc.perfect_sensors = true;
        sgs::sim::Trace trace{};
        sgs::sim::ScenarioMetrics metrics{};
        sgs::sim::run_scenario(seed, sc, sgs::default_gains(), &trace, &metrics);
        return metrics.max_err_settle;
      },
      py::arg("seed") = 1u, py::arg("duration_s") = 2.0);

  m.def(
      "trace_len_after_step_pitch",
      [](uint32_t seed) {
        sgs::sim::Scenario sc{};
        sc.kind = sgs::sim::ScenarioKind::StepPitch;
        sc.duration_s = 1.5;
        sgs::sim::Trace trace{};
        sgs::sim::run_scenario(seed, sc, sgs::default_gains(), &trace, nullptr);
        return static_cast<int>(trace.samples.size());
      },
      py::arg("seed") = 1u);
}
