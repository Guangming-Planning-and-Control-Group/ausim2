#pragma once

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

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

inline bool SendPacketBytes(int fd, const void* data, std::size_t size, bool non_blocking) {
  int flags = MSG_NOSIGNAL;
  if (non_blocking) {
    flags |= MSG_DONTWAIT;
  }

  const auto* bytes = static_cast<const std::uint8_t*>(data);
  for (;;) {
    const ssize_t written = send(fd, bytes, size, flags);
    if (written == static_cast<ssize_t>(size)) {
      return true;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
}

inline PacketReceiveStatus ReceivePacketBytes(int fd, std::vector<std::uint8_t>* buffer, std::size_t max_size) {
  if (buffer == nullptr || max_size == 0) {
    return PacketReceiveStatus::kError;
  }

  for (;;) {
    std::uint8_t probe = 0;
    iovec iov{&probe, sizeof(probe)};
    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    const ssize_t peeked = recvmsg(fd, &msg, MSG_PEEK | MSG_TRUNC);
    if (peeked > 0) {
      const std::size_t packet_size = static_cast<std::size_t>(peeked);
      if (packet_size > max_size) {
        recv(fd, &probe, sizeof(probe), 0);
        return PacketReceiveStatus::kError;
      }

      buffer->resize(packet_size);
      const ssize_t received = recv(fd, buffer->data(), buffer->size(), 0);
      if (received == static_cast<ssize_t>(buffer->size())) {
        return PacketReceiveStatus::kPacket;
      }
      if (received == 0) {
        return PacketReceiveStatus::kClosed;
      }
      return PacketReceiveStatus::kError;
    }
    if (peeked == 0) {
      return PacketReceiveStatus::kClosed;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
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
