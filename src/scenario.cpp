#include "hybrid_astar/scenario.hpp"

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace hybrid_astar {

namespace {

// Reads the next header token of a PGM, skipping '#' comments.
std::string pgmToken(std::istream& in) {
  std::string tok;
  while (in >> tok) {
    if (tok[0] != '#') return tok;
    std::string rest;
    std::getline(in, rest);
  }
  return {};
}

std::vector<double> parseNumbers(const std::string& text,
                                 const std::string& key,
                                 const std::string& file,
                                 size_t count) {
  std::istringstream ss(text);
  std::vector<double> out;
  double v;
  while (ss >> v) out.push_back(v);
  if (out.size() != count || !ss.eof()) {
    throw std::runtime_error(file + ": '" + key + "' needs " +
                             std::to_string(count) + " numbers");
  }
  return out;
}

} // namespace

GridMap loadPgm(const std::string& path, double resolution,
                double origin_x, double origin_y)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot open " + path);
  if (pgmToken(in) != "P5") throw std::runtime_error(path + ": not a binary PGM (P5)");
  int w = std::stoi(pgmToken(in));
  int h = std::stoi(pgmToken(in));
  int maxval = std::stoi(pgmToken(in));
  if (w <= 0 || h <= 0 || maxval != 255) {
    throw std::runtime_error(path + ": only 8-bit PGM is supported");
  }
  in.get();  // the single whitespace byte after maxval

  std::vector<unsigned char> pixels(size_t(w) * h);
  in.read(reinterpret_cast<char*>(pixels.data()), pixels.size());
  if (in.gcount() != std::streamsize(pixels.size())) {
    throw std::runtime_error(path + ": file is shorter than its header says");
  }

  GridMap grid;
  grid.width = w;
  grid.height = h;
  grid.resolution = resolution;
  grid.origin_x = origin_x;
  grid.origin_y = origin_y;
  grid.data.resize(pixels.size());
  // Image row 0 is the top of the map, grid row 0 is the bottom.
  for (int iy = 0; iy < h; ++iy) {
    const unsigned char* row = &pixels[size_t(h - 1 - iy) * w];
    for (int ix = 0; ix < w; ++ix) {
      grid.data[size_t(iy) * w + ix] = row[ix] < 128 ? 100 : 0;
    }
  }
  return grid;
}

Scenario loadScenario(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open " + path);

  std::map<std::string, std::string> kv;
  std::string line;
  int lineno = 0;
  while (std::getline(in, line)) {
    ++lineno;
    auto hash = line.find('#');
    if (hash != std::string::npos) line.erase(hash);
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    auto colon = line.find(':');
    if (colon == std::string::npos) {
      throw std::runtime_error(path + ":" + std::to_string(lineno) +
                               ": expected 'key: value'");
    }
    std::string key = line.substr(0, colon);
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    kv[key] = line.substr(colon + 1);
  }

  for (const char* k : {"map", "resolution", "origin", "start", "goal"}) {
    if (!kv.count(k)) throw std::runtime_error(path + ": missing '" + k + "'");
  }
  for (auto& [k, v] : kv) {
    if (k != "map" && k != "resolution" && k != "origin" &&
        k != "start" && k != "goal") {
      throw std::runtime_error(path + ": unknown key '" + k + "'");
    }
  }

  std::string map_file = kv["map"];
  map_file.erase(0, map_file.find_first_not_of(" \t"));
  map_file.erase(map_file.find_last_not_of(" \t\r") + 1);
  std::string dir;
  auto slash = path.find_last_of('/');
  if (slash != std::string::npos) dir = path.substr(0, slash + 1);

  double res = parseNumbers(kv["resolution"], "resolution", path, 1)[0];
  auto origin = parseNumbers(kv["origin"], "origin", path, 2);
  auto s = parseNumbers(kv["start"], "start", path, 3);
  auto g = parseNumbers(kv["goal"], "goal", path, 3);

  Scenario sc;
  sc.name = path.substr(slash == std::string::npos ? 0 : slash + 1);
  auto dot = sc.name.rfind('.');
  if (dot != std::string::npos) sc.name.erase(dot);
  sc.grid = loadPgm(dir + map_file, res, origin[0], origin[1]);
  sc.start = {s[0], s[1], s[2] * M_PI / 180.0};
  sc.goal = {g[0], g[1], g[2] * M_PI / 180.0};
  return sc;
}

} // namespace hybrid_astar
