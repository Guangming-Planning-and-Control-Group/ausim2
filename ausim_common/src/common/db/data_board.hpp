#pragma once

#include <any>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <typeinfo>
#include <unordered_map>

#include "common/db/security_data.hpp"

namespace ausim::db {

class DataBoardManager {
 public:
  DataBoardManager(const DataBoardManager&) = delete;
  DataBoardManager& operator=(const DataBoardManager&) = delete;
  DataBoardManager(DataBoardManager&&) = delete;
  DataBoardManager& operator=(DataBoardManager&&) = delete;

  static DataBoardManager& Instance() {
    static DataBoardManager instance;
    return instance;
  }

  template <typename T, Permission P = Permission::ReadOnly>
  SecurityDataRef<T, P> Get(std::string_view tag = {}) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string key = BuildKey<T>(tag);

    auto it = entries_.find(key);
    if (it == entries_.end()) {
      auto slot = std::make_shared<SecurityData<T>>();
      entries_.emplace(key, slot);
      return SecurityDataRef<T, P>(std::move(slot));
    }

    try {
      return SecurityDataRef<T, P>(std::any_cast<std::shared_ptr<SecurityData<T>>>(it->second));
    } catch (const std::bad_any_cast& error) {
      throw std::runtime_error("DataBoard type mismatch for key '" + key + "': " + error.what());
    }
  }

  template <typename T>
  SecurityDataRef<T, Permission::ReadOnly> Read(std::string_view tag = {}) {
    return Get<T, Permission::ReadOnly>(tag);
  }

  template <typename T>
  SecurityDataRef<T, Permission::ReadWrite> Write(std::string_view tag = {}) {
    return Get<T, Permission::ReadWrite>(tag);
  }

 private:
  DataBoardManager() = default;
  ~DataBoardManager() = default;

  template <typename T>
  static std::string BuildKey(std::string_view tag) {
    return std::string(typeid(T).name()) + "::" + std::string(tag);
  }

  std::mutex mutex_;
  std::unordered_map<std::string, std::any> entries_;
};

inline auto DataBoard = &DataBoardManager::Instance;

}  // namespace ausim::db
