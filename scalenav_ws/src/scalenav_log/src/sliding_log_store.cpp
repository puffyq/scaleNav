#include "scalenav_log/sliding_log_store.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace fs = std::filesystem;

namespace scalenav_log {

std::string jsonQuote(const std::string &value)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : value) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(c) << std::dec << std::setfill(' ');
        } else {
          out << static_cast<char>(c);
        }
    }
  }
  out << '"';
  return out.str();
}

std::string jsonNumber(const double value)
{
  if (!std::isfinite(value)) return "null";
  std::ostringstream out;
  out << std::setprecision(9) << value;
  return out.str();
}

SlidingLogStore::SlidingLogStore(fs::path root)
: root_(std::move(root)) {}

SlidingLogStore::~SlidingLogStore() { close(); }

std::string SlidingLogStore::timestampName()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&time, &local);
  std::ostringstream out;
  out << "session_" << std::put_time(&local, "%Y%m%d_%H%M%S") << "_"
      << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
  return out.str();
}

void SlidingLogStore::open(const std::string &manifest_json)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fs::create_directories(root_);
  manifest_json_ = manifest_json;
  openSessionLocked(manifest_json_);
}

void SlidingLogStore::openSessionLocked(const std::string &manifest_json)
{
  if (index_stream_) index_stream_->close();
  std::string name = timestampName();
  fs::path candidate = root_ / name;
  int suffix = 0;
  while (fs::exists(candidate)) candidate = root_ / (name + "_" + std::to_string(++suffix));
  session_dir_ = candidate;
  fs::create_directories(session_dir_ / "depth");
  fs::create_directories(session_dir_ / "pointcloud");
  fs::create_directories(session_dir_ / "graph");
  fs::create_directories(session_dir_ / "meta");
  index_path_ = session_dir_ / "index.jsonl";
  index_stream_ = std::make_unique<std::ofstream>(index_path_, std::ios::out | std::ios::app);
  if (!index_stream_->good()) throw std::runtime_error("cannot open log index: " + index_path_.string());
  std::ofstream manifest(session_dir_ / "manifest.json", std::ios::out | std::ios::trunc);
  manifest << manifest_json << '\n';
  manifest.close();
  sequence_ = 0;
}

void SlidingLogStore::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (index_stream_) {
    index_stream_->flush();
    index_stream_->close();
    index_stream_.reset();
  }
}

std::string SlidingLogStore::writeAsset(const std::string &relative_name,
                                        const std::vector<std::uint8_t> &bytes)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!index_stream_) throw std::runtime_error("log store is not open");
  fs::path relative(relative_name);
  if (relative.is_absolute() || std::any_of(relative.begin(), relative.end(),
      [](const fs::path &part) { return part == ".."; })) {
    throw std::invalid_argument("asset path must not escape the active session");
  }
  const fs::path destination = session_dir_ / relative;
  fs::create_directories(destination.parent_path());
  const fs::path temporary = destination.string() + ".tmp";
  {
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    if (!stream.good()) throw std::runtime_error("cannot write log asset: " + destination.string());
    if (!bytes.empty()) stream.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
  }
  fs::rename(temporary, destination);
  return relative.generic_string();
}

void SlidingLogStore::record(const std::string &kind, const std::int64_t stamp_ns,
                             const std::string &relative_file, const std::size_t bytes,
                             const std::string &extra_json)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!index_stream_) throw std::runtime_error("log store is not open");
  const std::string line = "{\"seq\":" + std::to_string(++sequence_) +
    ",\"kind\":" + jsonQuote(kind) +
    ",\"stamp_ns\":" + std::to_string(stamp_ns) +
    ",\"file\":" + jsonQuote(relative_file) +
    ",\"bytes\":" + std::to_string(bytes) +
    ",\"data\":" + (extra_json.empty() ? "{}" : extra_json) + "}\n";
  (*index_stream_) << line;
  index_stream_->flush();
}

std::filesystem::path SlidingLogStore::activeSession() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return session_dir_;
}

}  // namespace scalenav_log
