//
// Created by peter on 31/12/24.
//
#pragma once

#include <fmt/format.h>
#include <stdarg.h>
#include <queue>
#include <string>
#include "core.h"

typedef enum {
  LOG_ALL = 0,
  LOG_INFO,
  LOG_WARN,
  LOG_ERROR,
} LogLevel;

const int LOG_LINE_LENGTH = 100;
static LogLevel mLoggingLevel;
static char logLineBuffer[LOG_LINE_LENGTH];
const char *STR_INFO = "I: ";
const char *STR_WARN = "W: ";
const char *STR_ERROR = "E: ";

class Log {
 public:
  static void logSetLevel(LogLevel severity) {
    mLoggingLevel = severity;
  };

  static void add(const char *s) {
    std::string msg(s);
    add(msg);
  }

  static void add(std::string &msg) {
    msg = fmt::format("{:>8} ", g_ticks) + msg;
    logMessage(msg);
  }

  static void info(const char *format, ...) {
    if (LOG_INFO < mLoggingLevel) {
      return;
    }
    va_list args;
    va_start(args, format);
    vsnprintf(logLineBuffer, LOG_LINE_LENGTH, format, args);
    //  logAddString(STR_INFO);
    add(logLineBuffer);

    va_end(args);
  };

  static void warn(const char *format, ...) {
    if (LOG_WARN < mLoggingLevel) {
      return;
    }
    va_list args;
    va_start(args, format);
    vsnprintf(logLineBuffer, LOG_LINE_LENGTH, format, args);
    //  logAddString(STR_WARN);
    add(logLineBuffer);
    va_end(args);
  };

  static void error(const char *format, ...) {
    if (LOG_ERROR < mLoggingLevel) {
      return;
    }
    va_list args;
    va_start(args, format);
    vsnprintf(logLineBuffer, LOG_LINE_LENGTH, format, args);
    //  logAddString(STR_ERROR);
    add(logLineBuffer);
    va_end(args);
  }
};
