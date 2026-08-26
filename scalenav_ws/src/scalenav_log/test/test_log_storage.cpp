#include "scalenav_log/sliding_log_store.hpp"

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

TEST(ScalenavLogStore, WritesIndexWithoutRollingOrDeletingSessions)
{
  const auto root = fs::temp_directory_path() / "scalenav_log_store_test";
  std::error_code error;
  fs::remove_all(root, error);
  const auto old_session = root / "session_20200101_000000_000";
  fs::create_directories(old_session);
  std::ofstream(old_session / "sentinel.txt") << "keep";
  scalenav_log::SlidingLogStore store(root);
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
  EXPECT_TRUE(fs::exists(old_session / "sentinel.txt"));
  EXPECT_EQ(store.activeSession().filename().string().rfind("session_", 0), 0U);
  fs::remove_all(root, error);
}
