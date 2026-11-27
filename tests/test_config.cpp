#include "hybrid_astar/config.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace hybrid_astar;

TEST(Config, ParsesValuesAndComments) {
  auto cfg = parseConfig(
    "# a comment\n"
    "\n"
    "wheel_base: 2.7   # m\n"
    "heading_bins: 36\n"
    "   reverse_penalty :1.5\n");
  EXPECT_DOUBLE_EQ(cfg.wheel_base, 2.7);
  EXPECT_EQ(cfg.heading_bins, 36);
  EXPECT_DOUBLE_EQ(cfg.reverse_penalty, 1.5);
  // untouched keys keep their defaults
  EXPECT_DOUBLE_EQ(cfg.max_steer, PlannerConfig{}.max_steer);
}

TEST(Config, RejectsBadInput) {
  EXPECT_THROW(parseConfig("whel_base: 2.5\n"), std::runtime_error);
  EXPECT_THROW(parseConfig("wheel_base: 2.5m\n"), std::runtime_error);
  EXPECT_THROW(parseConfig("wheel_base:\n"), std::runtime_error);
  EXPECT_THROW(parseConfig("wheel_base 2.5\n"), std::runtime_error);
  EXPECT_THROW(parseConfig("wheel_base: -1\n"), std::runtime_error);
  EXPECT_THROW(parseConfig("heading_bins: 24.5\n"), std::runtime_error);
  EXPECT_THROW(parseConfig("wheel_base: 2\nwheel_base: 3\n"), std::runtime_error);
  EXPECT_THROW(parseConfig("hybrid_astar_node:\n  ros__parameters:\n"), std::runtime_error);
}

TEST(Config, ErrorNamesTheLine) {
  try {
    parseConfig("wheel_base: 2.5\nmax_steer: abc\n", "p.yaml");
    FAIL() << "no exception";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("p.yaml:2"), std::string::npos) << e.what();
  }
}

TEST(Config, RepoParamsFileLoads) {
  auto cfg = loadConfig(std::string(HA_SOURCE_DIR) + "/config/params.yaml");
  EXPECT_DOUBLE_EQ(cfg.wheel_base, 2.5);
  EXPECT_DOUBLE_EQ(cfg.max_steer, 0.45);
}

TEST(Config, EveryKeyIsInParamsFile) {
  // params.yaml should list every parameter, so the file is the full picture.
  std::ifstream in(std::string(HA_SOURCE_DIR) + "/config/params.yaml");
  std::stringstream ss;
  ss << in.rdbuf();
  for (const auto& k : configKeys()) {
    EXPECT_NE(ss.str().find("\n" + k + ":"), std::string::npos) << k;
  }
}
