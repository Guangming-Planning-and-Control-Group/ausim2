#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <unistd.h>

namespace ausim::ipc {

inline bool WriteFully(int fd, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  std::size_t total = 0;
  while (total < size) {
    const ssize_t written = write(fd, bytes + total, size - total);
    if (written > 0) {
      total += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

inline bool ReadFully(int fd, void* data, std::size_t size) {
  auto* bytes = static_cast<std::uint8_t*>(data);
  std::size_t total = 0;
  while (total < size) {
    const ssize_t received = read(fd, bytes + total, size - total);
    if (received > 0) {
      total += static_cast<std::size_t>(received);
      continue;
    }
    if (received == 0) {
      return false;
    }
    if (errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

}  // namespace ausim::ipc
