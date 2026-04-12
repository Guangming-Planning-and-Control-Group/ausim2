#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <sys/socket.h>
#include <type_traits>
#include <unistd.h>

namespace ausim::ipc {

enum class PacketReceiveStatus {
  kPacket,
  kClosed,
  kWouldBlock,
  kError,
};

inline bool SetNonBlocking(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

template <typename Packet>
bool SendPacket(int fd, const Packet& packet, bool non_blocking) {
  static_assert(std::is_trivially_copyable_v<Packet>);

  int flags = MSG_NOSIGNAL;
  if (non_blocking) {
    flags |= MSG_DONTWAIT;
  }

  for (;;) {
    const ssize_t bytes = send(fd, &packet, sizeof(Packet), flags);
    if (bytes == static_cast<ssize_t>(sizeof(Packet))) {
      return true;
    }
    if (bytes < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
}

template <typename Packet>
PacketReceiveStatus ReceivePacket(int fd, Packet* packet) {
  static_assert(std::is_trivially_copyable_v<Packet>);

  for (;;) {
    const ssize_t bytes = recv(fd, packet, sizeof(Packet), 0);
    if (bytes == static_cast<ssize_t>(sizeof(Packet))) {
      return PacketReceiveStatus::kPacket;
    }
    if (bytes == 0) {
      return PacketReceiveStatus::kClosed;
    }
    if (bytes < 0 && errno == EINTR) {
      continue;
    }
    if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return PacketReceiveStatus::kWouldBlock;
    }
    return PacketReceiveStatus::kError;
  }
}

inline void ShutdownAndClose(int* fd) {
  if (fd == nullptr || *fd < 0) {
    return;
  }
  shutdown(*fd, SHUT_RDWR);
  close(*fd);
  *fd = -1;
}

}  // namespace ausim::ipc
