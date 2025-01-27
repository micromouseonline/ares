//
// Created by peter on 31/12/24.
//
#pragma once

#include <stdarg.h>
#include <queue>
#include <string>
#include "common/core.h"
#include "common/printf/printf.h"

typedef enum {
  LOG_ALL = 0,
  LOG_TRACE,
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

class MouseLog {
 public:
  void logSetLevel(LogLevel severity) {
    mLoggingLevel = severity;
  };

  void log(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf_(logLineBuffer, LOG_LINE_LENGTH, format, args);
    add(logLineBuffer);
    va_end(args);
  };

  void trace(const char *format, ...) {
    if (LOG_TRACE < mLoggingLevel) {
      return;
    }
    va_list args;
    va_start(args, format);
    vsnprintf_(logLineBuffer, LOG_LINE_LENGTH, format, args);
    add(logLineBuffer);
    va_end(args);
  };

  void info(const char *format, ...) {
    if (LOG_INFO < mLoggingLevel) {
      return;
    }
    va_list args;
    va_start(args, format);
    vsnprintf_(logLineBuffer, LOG_LINE_LENGTH, format, args);
    add(logLineBuffer);
    va_end(args);
  };

  void warn(const char *format, ...) {
    if (LOG_WARN < mLoggingLevel) {
      return;
    }
    va_list args;
    va_start(args, format);
    vsnprintf_(logLineBuffer, LOG_LINE_LENGTH, format, args);
    add(logLineBuffer);
    va_end(args);
  };

  void error(const char *format, ...) {
    if (LOG_ERROR < mLoggingLevel) {
      return;
    }
    va_list args;
    va_start(args, format);
    vsnprintf_(logLineBuffer, LOG_LINE_LENGTH, format, args);
    //  logAddString(STR_ERROR);
    add(logLineBuffer);
    va_end(args);
  }

 private:
  void add(const char *s) {
    std::string msg(s);
    add(msg);
  }

  void add(std::string &msg) {
    logMessage(msg);
  }
};
