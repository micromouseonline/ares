//
// Created by peter on 01/01/25.
//
#pragma once

#include <spdlog/spdlog.h>
#include <memory>
#include "spdlog/sinks/stdout_color_sinks.h"

#include "spdlog/spdlog.h"

#define LOGGER_NAME "ARES"

#if defined(ARES_PLATFORM_WINDOWS)
#define ARES_BREAK __debugbreak();
#elif defined(ARES_PLATFORM_MAC)
#define ARES_BREAK __builtin_debugtrap();
#else
#define ASSERT_BREAK __builtin_trap();
#endif

#ifndef ARES_CONFIG_RELEASE
#define ARES_TRACE(...)                           \
  if (spdlog::get(LOGGER_NAME) != nullptr) {      \
    spdlog::get(LOGGER_NAME)->trace(__VA_ARGS__); \
  }
#define ARES_DEBUG(...)                           \
  if (spdlog::get(LOGGER_NAME) != nullptr) {      \
    spdlog::get(LOGGER_NAME)->debug(__VA_ARGS__); \
  }
#define ARES_INFO(...)                           \
  if (spdlog::get(LOGGER_NAME) != nullptr) {     \
    spdlog::get(LOGGER_NAME)->info(__VA_ARGS__); \
  }
#define ARES_WARN(...)                           \
  if (spdlog::get(LOGGER_NAME) != nullptr) {     \
    spdlog::get(LOGGER_NAME)->warn(__VA_ARGS__); \
  }
#define ARES_ERROR(...)                           \
  if (spdlog::get(LOGGER_NAME) != nullptr) {      \
    spdlog::get(LOGGER_NAME)->error(__VA_ARGS__); \
  }
#define ARES_FATAL(...)                              \
  if (spdlog::get(LOGGER_NAME) != nullptr) {         \
    spdlog::get(LOGGER_NAME)->critical(__VA_ARGS__); \
  }
#define ARES_ASSERT(x, msg)                                                                     \
  if ((x)) {                                                                                    \
  } else {                                                                                      \
    ARES_FATAL("ASSERT - {}\n\t{}\n\tin file: {}\n\ton line: {}", #x, msg, __FILE__, __LINE__); \
    ASSERT_BREAK                                                                                \
  }
#else
// Disable logging for release builds
#define ARES_TRACE(...) (void)0
#define ARES_DEBUG(...) (void)0
#define ARES_INFO(...) (void)0
#define ARES_WARN(...) (void)0
#define ARES_ERROR(...) (void)0
#define ARES_FATAL(...) (void)0
#define ARES_ASSERT(x, msg) (void)0
#endif

#include <memory>
class LogManager {
 public:
  LogManager() = default;
  ~LogManager() = default;

  void initialise() {
    auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    consoleSink->set_pattern("%^[%Y-%m-%d %H:%M:%S.%e] %v%$");

    std::vector<spdlog::sink_ptr> sinks{consoleSink};
    auto logger = std::make_shared<spdlog::logger>(LOGGER_NAME, sinks.begin(), sinks.end());
    logger->set_level(spdlog::level::trace);
    logger->flush_on(spdlog::level::trace);
    spdlog::register_logger(logger);
  }

  void Shutdown() {
    spdlog::shutdown();
  }
};
