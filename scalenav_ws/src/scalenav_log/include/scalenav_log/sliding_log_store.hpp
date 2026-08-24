#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace scalenav_log {

// A session is append-only while it is active. Completed/older sessions are
// removed oldest-first when the configured byte or session limits are crossed.
class SlidingLogStore {
public:
  SlidingLogStore(std::filesystem::path root, std::uint64_t max_total_bytes,
                  std::size_t max_sessions, std::uint64_t max_session_bytes);
  ~SlidingLogStore();

  void open(const std::string &manifest_json);
  void close();
  void prune();

  // Writes an asset below the active session and returns its session-relative
  // URL, e.g. "depth/depth_000001.pgm".
  std::string writeAsset(const std::string &relative_name,
                         const std::vector<std::uint8_t> &bytes);

  // extra_json must be a JSON object without the surrounding index fields.
  void record(const std::string &kind, std::int64_t stamp_ns,
              const std::string &relative_file, std::size_t bytes,
              const std::string &extra_json = "{}");

  std::filesystem::path root() const { return root_; }
  std::filesystem::path activeSession() const;
  std::uint64_t activeBytes() const;

private:
  void openSessionLocked(const std::string &manifest_json);
  void pruneLocked();
  std::uint64_t directoryBytes(const std::filesystem::path &path) const;
  static std::string timestampName();

  std::filesystem::path root_;
  std::uint64_t max_total_bytes_;
  std::size_t max_sessions_;
  std::uint64_t max_session_bytes_;
  mutable std::mutex mutex_;
  std::filesystem::path session_dir_;
  std::filesystem::path index_path_;
  std::uint64_t active_bytes_ = 0;
  std::uint64_t sequence_ = 0;
  std::string manifest_json_;
  std::unique_ptr<std::ofstream> index_stream_;
};

std::string jsonQuote(const std::string &value);
std::string jsonNumber(double value);

}  // namespace scalenav_log
