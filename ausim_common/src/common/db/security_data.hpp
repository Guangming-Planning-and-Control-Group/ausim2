#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <type_traits>
#include <utility>

namespace ausim::db {

enum class Permission { ReadOnly, ReadWrite };

template <typename T>
class SecurityData {
 public:
  void Write(const T& value) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    value_ = value;
  }

  void Write(T&& value) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    value_ = std::move(value);
  }

  std::optional<T> ReadOptional() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return value_;
  }

  bool IsValid() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return value_.has_value();
  }

  template <typename Fn>
  void Update(Fn&& fn) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    if (!value_.has_value()) {
      value_.emplace();
    }
    std::forward<Fn>(fn)(*value_);
  }

 private:
  mutable std::shared_mutex mutex_;
  std::optional<T> value_;
};

template <typename T, Permission Perm>
class SecurityDataRef {
 public:
  explicit SecurityDataRef(std::shared_ptr<SecurityData<T>> data = nullptr) : data_(std::move(data)) {}

  explicit operator bool() const { return data_ != nullptr; }

  bool IsValid() const { return data_ != nullptr && data_->IsValid(); }

  std::optional<T> ReadOptional() const { return data_ == nullptr ? std::nullopt : data_->ReadOptional(); }

  T operator()() const {
    const std::optional<T> value = ReadOptional();
    return value.has_value() ? *value : T{};
  }

  T operator*() const { return operator()(); }

  template <Permission P = Perm, typename std::enable_if_t<P == Permission::ReadWrite, int> = 0>
  void operator=(const T& value) const {
    if (data_ != nullptr) {
      data_->Write(value);
    }
  }

  template <Permission P = Perm, typename std::enable_if_t<P == Permission::ReadWrite, int> = 0>
  void operator=(T&& value) const {
    if (data_ != nullptr) {
      data_->Write(std::move(value));
    }
  }

  template <Permission P = Perm, typename Fn, typename std::enable_if_t<P == Permission::ReadWrite, int> = 0>
  void Update(Fn&& fn) const {
    if (data_ != nullptr) {
      data_->Update(std::forward<Fn>(fn));
    }
  }

 private:
  std::shared_ptr<SecurityData<T>> data_;
};

}  // namespace ausim::db
