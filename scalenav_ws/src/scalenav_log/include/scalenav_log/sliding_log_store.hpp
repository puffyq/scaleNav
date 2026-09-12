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

// A session is append-only while it is active. Sessions are created only when
// the logger starts; the store never rolls or deletes older sessions.
class SlidingLogStore {
public:
  explicit SlidingLogStore(std::filesystem::path root);
  ~SlidingLogStore();

  void open(const std::string &manifest_json);
  void close();

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

private:
  void openSessionLocked(const std::string &manifest_json);
  static std::string timestampName();

  std::filesystem::path root_;
  mutable std::mutex mutex_;
  std::filesystem::path session_dir_;
  std::filesystem::path index_path_;
  std::uint64_t sequence_ = 0;
  std::string manifest_json_;
  std::unique_ptr<std::ofstream> index_stream_;
};

std::string jsonQuote(const std::string &value);
std::string jsonNumber(double value);

}  // namespace scalenav_log
