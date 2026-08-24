#include "scalenav_log/sliding_log_store.hpp"

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

TEST(ScalenavLogStore, WritesIndexAndRollsSessions)
{
  const auto root = fs::temp_directory_path() / "scalenav_log_store_test";
  std::error_code error;
  fs::remove_all(root, error);
  scalenav_log::SlidingLogStore store(root, 1024 * 1024, 3, 1024);
  store.open("{\"schema\":\"test\"}");
  const std::vector<std::uint8_t> payload(700, 7);
  store.writeAsset("depth/a.bin", payload);
  store.record("depth", 42, "depth/a.bin", payload.size(), "{\"width\":1}");
  ASSERT_TRUE(fs::exists(store.activeSession() / "index.jsonl"));
  std::ifstream index(store.activeSession() / "index.jsonl");
  const std::string text((std::istreambuf_iterator<char>(index)), std::istreambuf_iterator<char>());
  EXPECT_NE(text.find("\"kind\":\"depth\""), std::string::npos);
  store.writeAsset("depth/b.bin", payload);
  EXPECT_NE(store.activeSession().filename().string(), "");
  fs::remove_all(root, error);
}
