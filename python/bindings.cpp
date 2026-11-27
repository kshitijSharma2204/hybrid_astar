// Python module "hybrid_astar". Thin glue over the C++ library.

#include "hybrid_astar/config.hpp"
#include "hybrid_astar/planner.hpp"
#include "hybrid_astar/scenario.hpp"

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <optional>

namespace py = pybind11;
using namespace hybrid_astar;

namespace {

using Int8Array = py::array_t<int8_t, py::array::c_style | py::array::forcecast>;

GridMap gridFromArray(Int8Array a, double resolution, std::pair<double, double> origin) {
  if (a.ndim() != 2) throw py::value_error("grid must be a 2D array (rows, cols)");
  if (resolution <= 0) throw py::value_error("resolution must be > 0");
  GridMap g;
  g.height = int(a.shape(0));
  g.width = int(a.shape(1));
  g.resolution = resolution;
  g.origin_x = origin.first;
  g.origin_y = origin.second;
  g.data.assign(a.data(), a.data() + a.size());
  return g;
}

Int8Array gridToArray(const GridMap& g) {
  Int8Array a({g.height, g.width});
  std::memcpy(a.mutable_data(), g.data.data(), g.data.size());
  return a;
}

template <typename T, typename F>
py::array_t<T> column(size_t n, F f) {
  py::array_t<T> a(n);
  auto m = a.template mutable_unchecked<1>();
  for (size_t i = 0; i < n; ++i) m(i) = f(i);
  return a;
}

GridMap toGrid(py::object grid, double resolution, std::pair<double, double> origin) {
  if (py::isinstance<GridMap>(grid)) return grid.cast<GridMap>();
  if (resolution <= 0) throw py::value_error("pass resolution= when grid is a numpy array");
  return gridFromArray(grid.cast<Int8Array>(), resolution, origin);
}

py::array_t<double> traceNodes(const SearchTrace& t, size_t from, size_t to) {
  py::array_t<double> a({to - from, size_t(4)});
  auto m = a.template mutable_unchecked<2>();
  for (size_t i = from; i < to; ++i) {
    const auto& n = t.nodes[i];
    m(i - from, 0) = n.x;
    m(i - from, 1) = n.y;
    m(i - from, 2) = n.theta;
    m(i - from, 3) = n.parent;
  }
  return a;
}

PlanResult runPlan(HybridAStarPlanner& planner, const Pose2D& start, const Pose2D& goal,
                   bool record_trace, py::object on_progress, int every) {
  PlanOptions opts;
  opts.record_trace = record_trace || !on_progress.is_none();
  opts.progress_every = every;
  size_t sent = 0;
  if (!on_progress.is_none()) {
    // The search runs without the GIL; take it only for the callback.
    opts.on_progress = [&](const SearchTrace& t) {
      py::gil_scoped_acquire gil;
      on_progress(traceNodes(t, sent, t.nodes.size()));
      sent = t.nodes.size();
    };
  }
  PlanResult r;
  {
    py::gil_scoped_release no_gil;
    r = planner.plan(start, goal, opts);
  }
  if (!record_trace) r.trace.reset();
  return r;
}

} // namespace

PYBIND11_MODULE(hybrid_astar, m) {
  m.doc() = "Hybrid A* path planner (C++), see README.md";

  py::class_<Pose2D>(m, "Pose2D")
    .def(py::init<>())
    .def(py::init([](double x, double y, double theta) { return Pose2D{x, y, theta}; }),
         py::arg("x"), py::arg("y"), py::arg("theta"))
    .def(py::init([](py::tuple t) {
      if (t.size() != 3) throw py::value_error("pose tuple must be (x, y, theta)");
      return Pose2D{t[0].cast<double>(), t[1].cast<double>(), t[2].cast<double>()};
    }))
    .def_readwrite("x", &Pose2D::x)
    .def_readwrite("y", &Pose2D::y)
    .def_readwrite("theta", &Pose2D::theta)
    .def("__iter__", [](const Pose2D& p) {
      return py::iter(py::make_tuple(p.x, p.y, p.theta));
    })
    .def("__repr__", [](const Pose2D& p) {
      return "Pose2D(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " +
             std::to_string(p.theta) + ")";
    });
  py::implicitly_convertible<py::tuple, Pose2D>();

  py::class_<GridMap>(m, "GridMap")
    .def(py::init([](Int8Array data, double resolution, std::pair<double, double> origin) {
           return gridFromArray(data, resolution, origin);
         }),
         py::arg("data"), py::arg("resolution"), py::arg("origin") = std::make_pair(0.0, 0.0),
         "data[row, col], row 0 at the bottom (smallest y), 100 = occupied")
    .def_readonly("width", &GridMap::width)
    .def_readonly("height", &GridMap::height)
    .def_readonly("resolution", &GridMap::resolution)
    .def_property_readonly("origin", [](const GridMap& g) {
      return py::make_tuple(g.origin_x, g.origin_y);
    })
    .def_property_readonly("data", &gridToArray, "copy of the cells as int8[height, width]")
    .def_property_readonly("extent", [](const GridMap& g) {
      return py::make_tuple(g.origin_x, g.origin_x + g.width * g.resolution,
                            g.origin_y, g.origin_y + g.height * g.resolution);
    }, "(xmin, xmax, ymin, ymax) for matplotlib imshow");

  py::class_<PlannerConfig> cfg(m, "PlannerConfig");
  cfg.def(py::init<>())
    .def("set", &setConfigValue, py::arg("key"), py::arg("value"),
         "set a parameter by name, with the same checks as params.yaml")
    .def("min_turning_radius", &PlannerConfig::minTurningRadius)
    .def_static("keys", &configKeys);
  // Every field as a property. Writes are checked like params.yaml values.
  for (const auto& key : configKeys()) {
    cfg.def_property(key.c_str(),
      [key](const PlannerConfig& c) { return getConfigValue(c, key); },
      [key](PlannerConfig& c, double v) { setConfigValue(c, key, v); });
  }

  py::class_<PlanStats>(m, "PlanStats")
    .def_readonly("success", &PlanStats::success)
    .def_readonly("expansions", &PlanStats::expansions)
    .def_readonly("search_ms", &PlanStats::search_ms)
    .def_readonly("smooth_ms", &PlanStats::smooth_ms);

  py::class_<SearchTrace::RsAttempt>(m, "RsAttempt")
    .def_readonly("node", &SearchTrace::RsAttempt::node)
    .def_readonly("collided", &SearchTrace::RsAttempt::collided)
    .def_property_readonly("samples", [](const SearchTrace::RsAttempt& a) {
      py::array_t<double> out({a.samples.size(), size_t(3)});
      auto m = out.mutable_unchecked<2>();
      for (size_t i = 0; i < a.samples.size(); ++i) {
        m(i, 0) = a.samples[i].x;
        m(i, 1) = a.samples[i].y;
        m(i, 2) = a.samples[i].theta;
      }
      return out;
    }, "float64[n, 3]: x, y, theta");

  py::class_<SearchTrace>(m, "SearchTrace")
    .def_property_readonly("nodes", [](const SearchTrace& t) {
      return traceNodes(t, 0, t.nodes.size());
    }, "float64[n, 4]: x, y, theta, parent index (-1 for the start)")
    .def_readonly("rs_attempts", &SearchTrace::rs_attempts);

  py::class_<PlanResult>(m, "PlanResult")
    .def_readonly("trace", &PlanResult::trace)
    .def_property_readonly("success", [](const PlanResult& r) { return r.stats.success; })
    .def_readonly("stats", &PlanResult::stats)
    .def_readonly("message", &PlanResult::message)
    .def_property_readonly("x", [](const PlanResult& r) {
      return column<double>(r.poses.size(), [&](size_t i) { return r.poses[i].x; });
    })
    .def_property_readonly("y", [](const PlanResult& r) {
      return column<double>(r.poses.size(), [&](size_t i) { return r.poses[i].y; });
    })
    .def_property_readonly("theta", [](const PlanResult& r) {
      return column<double>(r.poses.size(), [&](size_t i) { return r.poses[i].theta; });
    })
    .def_property_readonly("direction", [](const PlanResult& r) {
      return column<int8_t>(r.direction.size(), [&](size_t i) { return int8_t(r.direction[i]); });
    });

  py::class_<Scenario>(m, "Scenario")
    .def_readonly("name", &Scenario::name)
    .def_readonly("grid", &Scenario::grid)
    .def_readonly("start", &Scenario::start)
    .def_readonly("goal", &Scenario::goal);

  py::class_<HybridAStarPlanner>(m, "Planner",
      "Keeps the collision lookups for one map, for planning many times on it.")
    .def(py::init<const PlannerConfig&>(), py::arg("config") = PlannerConfig{})
    .def("set_map",
         [](HybridAStarPlanner& p, py::object grid, double resolution,
            std::pair<double, double> origin) {
           GridMap g = toGrid(grid, resolution, origin);
           py::gil_scoped_release no_gil;
           p.setMap(g);
         },
         py::arg("grid"), py::arg("resolution") = 0.0,
         py::arg("origin") = std::make_pair(0.0, 0.0))
    .def("plan",
         [](HybridAStarPlanner& p, const Pose2D& start, const Pose2D& goal,
            bool record_trace, py::object on_progress, int every) {
           return runPlan(p, start, goal, record_trace, on_progress, every);
         },
         py::arg("start"), py::arg("goal"), py::arg("record_trace") = false,
         py::arg("on_progress") = py::none(), py::arg("every") = 1000)
    .def_property_readonly("setup_ms", &HybridAStarPlanner::setupMs);

  m.def("load_config", &loadConfig, py::arg("path"));
  m.def("load_scenario", &loadScenario, py::arg("path"));

  m.def("plan",
    [](py::object grid, const Pose2D& start, const Pose2D& goal,
       std::optional<PlannerConfig> config, bool record_trace,
       double resolution, std::pair<double, double> origin,
       py::object on_progress, int every) {
      HybridAStarPlanner planner(config ? *config : PlannerConfig{});
      planner.setMap(toGrid(grid, resolution, origin));
      return runPlan(planner, start, goal, record_trace, on_progress, every);
    },
    py::arg("grid"), py::arg("start"), py::arg("goal"), py::arg("config") = py::none(),
    py::arg("record_trace") = false, py::arg("resolution") = 0.0,
    py::arg("origin") = std::make_pair(0.0, 0.0), py::arg("on_progress") = py::none(),
    py::arg("every") = 1000,
    R"(Plan from start to goal.

grid is a GridMap, or an int8 array (row 0 = smallest y) with resolution=.
With record_trace=True the result has .trace. on_progress(new_nodes) is
called every `every` expansions with the nodes expanded since the last
call, as float64[n, 4] (x, y, theta, parent).)");
}
