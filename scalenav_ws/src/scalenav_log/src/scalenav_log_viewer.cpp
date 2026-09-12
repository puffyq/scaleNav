#include "scalenav_log/sliding_log_store.hpp"

#include <arpa/inet.h>
#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;
using scalenav_log::jsonQuote;

namespace {

std::string mimeType(const fs::path &path)
{
  const auto extension = path.extension().string();
  if (extension == ".html") return "text/html; charset=utf-8";
  if (extension == ".js") return "text/javascript; charset=utf-8";
  if (extension == ".css") return "text/css; charset=utf-8";
  if (extension == ".json" || extension == ".jsonl") return "application/json; charset=utf-8";
  if (extension == ".pcd") return "text/plain; charset=utf-8";
  if (extension == ".pgm") return "image/x-portable-graymap";
  if (extension == ".png") return "image/png";
  return "application/octet-stream";
}

bool safeRelative(const fs::path &path)
{
  if (path.is_absolute()) return false;
  for (const auto &part : path) if (part == "..") return false;
  return true;
}

std::string readText(const fs::path &path)
{
  std::ifstream stream(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

std::string decodePath(std::string path)
{
  const auto query = path.find('?');
  if (query != std::string::npos) path.resize(query);
  std::string result;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (path[i] == '%' && i + 2 < path.size()) {
      const auto hex = path.substr(i + 1, 2);
      char *end = nullptr;
      const long value = std::strtol(hex.c_str(), &end, 16);
      if (end && *end == '\0') { result.push_back(static_cast<char>(value)); i += 2; continue; }
    }
    result.push_back(path[i]);
  }
  return result;
}

class HttpServer {
public:
  HttpServer(fs::path root, fs::path web_root, int port)
  : root_(std::move(root)), web_root_(std::move(web_root)), port_(port) {}

  int run()
  {
    const int server = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server < 0) { std::cerr << "socket: " << std::strerror(errno) << '\n'; return 1; }
    int reuse = 1;
    ::setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<std::uint16_t>(port_));
    if (::bind(server, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 ||
        ::listen(server, 16) < 0) {
      std::cerr << "bind/listen: " << std::strerror(errno) << '\n';
      ::close(server);
      return 1;
    }
    std::cout << "scalenav log viewer: http://127.0.0.1:" << port_ << "\n";
    while (true) {
      const int client = ::accept(server, nullptr, nullptr);
      if (client < 0) continue;
      handle(client);
      ::close(client);
    }
  }

private:
  void handle(const int client)
  {
    char buffer[8192]{};
    const ssize_t received = ::recv(client, buffer, sizeof(buffer) - 1, 0);
    if (received <= 0) return;
    std::istringstream request(std::string(buffer, static_cast<std::size_t>(received)));
    std::string method, target, version;
    request >> method >> target >> version;
    if (method != "GET") { respond(client, 405, "text/plain", "method not allowed\n"); return; }
    target = decodePath(target);
    if (target == "/api/sessions") { respond(client, 200, "application/json; charset=utf-8", sessionsJson()); return; }
    fs::path file;
    if (target == "/" || target.empty()) {
      file = web_root_ / "index.html";
    } else if (target.rfind("/web/", 0) == 0) {
      const fs::path relative = target.substr(5);
      if (!safeRelative(relative)) { respond(client, 400, "text/plain", "bad path\n"); return; }
      file = web_root_ / relative;
    } else if (target.rfind("/sessions/", 0) == 0) {
      const std::string remainder = target.substr(10);
      const auto slash = remainder.find('/');
      if (slash == std::string::npos) { respond(client, 404, "text/plain", "not found\n"); return; }
      const fs::path session = remainder.substr(0, slash);
      const fs::path relative = remainder.substr(slash + 1);
      if (!safeRelative(session) || !safeRelative(relative) || session.string().find("session_") != 0) {
        respond(client, 400, "text/plain", "bad path\n"); return;
      }
      file = root_ / session / relative;
    } else {
      respond(client, 404, "text/plain", "not found\n"); return;
    }
    std::error_code error;
    if (!fs::is_regular_file(file, error)) { respond(client, 404, "text/plain", "not found\n"); return; }
    std::ifstream stream(file, std::ios::binary);
    std::vector<char> body((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    respond(client, 200, mimeType(file), body.data(), body.size());
  }

  std::string sessionsJson() const
  {
    std::vector<fs::path> sessions;
    std::error_code error;
    for (const auto &entry : fs::directory_iterator(root_, error)) {
      if (!error && entry.is_directory(error) && entry.path().filename().string().rfind("session_", 0) == 0)
        sessions.push_back(entry.path());
    }
    std::sort(sessions.begin(), sessions.end(), [](const fs::path &a, const fs::path &b) { return a.filename() > b.filename(); });
    std::ostringstream out;
    out << "[";
    for (std::size_t i = 0; i < sessions.size(); ++i) {
      if (i) out << ',';
      std::uint64_t bytes = 0;
      for (const auto &item : fs::recursive_directory_iterator(sessions[i], error)) if (!error && item.is_regular_file(error)) bytes += item.file_size(error);
      out << "{\"name\":" << jsonQuote(sessions[i].filename().string()) << ",\"bytes\":" << bytes << "}";
    }
    out << "]";
    return out.str();
  }

  void respond(int client, int status, const std::string &type, const std::string &body)
  { respond(client, status, type, body.data(), body.size()); }

  void respond(int client, int status, const std::string &type, const char *body, std::size_t size)
  {
    const char *reason = status == 200 ? "OK" : status == 400 ? "Bad Request" : status == 404 ? "Not Found" : "Method Not Allowed";
    std::ostringstream header;
    header << "HTTP/1.1 " << status << ' ' << reason << "\r\nContent-Type: " << type
      << "\r\nContent-Length: " << size << "\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n";
    const auto text = header.str();
    ::send(client, text.data(), text.size(), MSG_NOSIGNAL);
    if (size) ::send(client, body, size, MSG_NOSIGNAL);
  }

  fs::path root_, web_root_;
  int port_;
};

}  // namespace

int main(int argc, char **argv)
{
  fs::path root = "./scalenav_logs";
  fs::path web_root;
  int port = 8765;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--root" && i + 1 < argc) root = argv[++i];
    else if (arg == "--web-root" && i + 1 < argc) web_root = argv[++i];
    else if (arg == "--port" && i + 1 < argc) port = std::stoi(argv[++i]);
  }
  if (web_root.empty()) {
    const char *prefix = std::getenv("AMENT_PREFIX_PATH");
    if (prefix) web_root = fs::path(std::string(prefix).substr(0, std::string(prefix).find(':'))) / "share/scalenav_log/web";
  }
  if (web_root.empty() || !fs::exists(web_root)) web_root = fs::path("web");
  return HttpServer(root, web_root, port).run();
}
