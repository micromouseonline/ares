//
// Created by peter on 12/01/25.
//

#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "commands.h"
#include "common/line-processor.h"
#include "common/timer.h"
#include "target.h"

// Manager class
class Manager {
 public:
  const int OUTPUT_QUEUE_SIZE = 2048;
  Target target;
  std::thread target_thread;
  Queue<char> target_serial_out;
  std::atomic<bool> paused;
  /// we need a mutex for access into the target
  std::mutex target_mutex;
  /// and a mutex just for the logging queue
  std::mutex log_mutex;
  std::condition_variable pauseCV;
  std::vector<std::string> target_log;
  LineProcessor processor;

 public:
  Manager() : target(), target_serial_out(OUTPUT_QUEUE_SIZE), target_mutex(), log_mutex(), processor() {
    printf("Manager created\n");
    RunTarget();
  }

  ~Manager() {
    printf("Stop the target\n");
    stopTarget();
    printf("Join the target thread\n");
    if (target_thread.joinable()) {
      target_thread.join();
    }
    printf("Manager destroyed\n");
  }

  Target& getTarget() {
    return target;
  }

  void RunTarget() {
    target.setLogCallback([this](const char* message) { this->targetSerialOutCallback(message); });
    printf("Start the target thread\n");
    target_thread = std::thread([this]() { target.mainLoop(); });
  }
  ////////////////////////////////////
  void stopTarget() {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.stopRunning();
  }

  void pauseTarget() {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.pauseRunning();
  }

  void resumeTarget() {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.resumeRunning();
  }
  ////////////////////////////////////

  void setParameter(float p) {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.setFilterAlpha(p);
  }

  float getParameter() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.getFilterAlpha();
  }

  int processOutput() {
    std::lock_guard<std::mutex> lock(log_mutex);
    int count = target_serial_out.size();
    processor.processQueue(target_serial_out, target_log);
    return count;
  }

  void targetSerialOutCallback(const char* msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    const char* c = msg;
    while (*c) {
      target_serial_out.push(*c);
      c++;
    }
    target_serial_out.push('\n');
    target_serial_out.push('\0');
  }

  /***
   * Mocking up the serial IO is a bit of a challenge. On a typical
   * microcontroller it is easy and common to be able to redefine
   * the character IO functions _read() and _write(). This is not
   * easy on a PC and even less easy to make platform independant.
   *
   * Thus the target code will need to use custom character IO
   * functions. This is common enough so should not be a problem.
   *
   * Either redefine _read() and _write() - or their equivalents - or
   * make sure the custom character drivers can have their lowest
   * level function point to a corresponding pair of functions.
   *
   * When running on the target hardware, those functions will usually
   * read from or write to the serial driver buffers. Special drivers
   * may write to other devices like SPI for SD cards or Flash chips.
   * Reading and writing characters can be done with file IDs using
   * fprintf() and fscanf(). The file IDs can identify different
   * destinations.
   *
   * However it gets done, the aim is to have low-level character driver
   * functions that can be redirected to use callbacks in the manager
   * when the code is running under simulation.
   *
   * Arranged this way, the target code need know almost nothing about
   * its environment. Conditional compilation can call the appropriate
   * functions. Even the setting of the calbacks can be used perfectly
   * well on the target hardware since the same mechanism could be used
   * to redirect character IO to different devices as the robot runs.
   *
   */

  /***
   * serialWriteCallback will be called by the target's _write() function.
   * _write() is the end of the chain called by printf, putchar and putc.
   * All character output eventually ends up there.
   * NOTE: check with specific toolchain to see what is needed.
   * A typical _write has a file id as its firct parameter. We can use that
   * to direct output through this function if desired. On the target the
   * _write function might look like this:
   *
   *       // Override the _write function
   *       extern "C" int _write(int file, const char* data, int length) {
   *       // For "standard output" (stdout) and "standard error" (stderr)
   *       if (file == STDOUT_FILENO || file == STDERR_FILENO) {
   *          Serial_Write(data, length);
   *          return length; // Return the number of bytes written
   *       }
   *       return -1; // Error condition
   *       }
   *
   * For our purposes, we might pick a particular fileID and write to the
   * appropriate callback function to put characters into the manager's queue.
   * This makes things transparent to the target and thread-safe in the manager.
   *
   * @param data
   * @param length
   */
  void serialWriteCallback(const char* data, int length) {
    std::lock_guard<std::mutex> lock(log_mutex);
    for (int i = 0; i < length; i++) {
      target_serial_out.push(data[i]);
    }
  }

  /***
   * serialReadCallback will be called by the target's _read() function.
   * _read() is the end of the chain called by scanf, getchar and getc.
   * All character input eventually ends up there.
   * NOTE: check with specific toolchain to see what is needed.
   * A typical _read has a file id as its firct parameter. We can use that
   * to direct input through this function if desired. On the target the
   * _read function might look like this:
   *
   *    // Override the _read function
   *    extern "C" int _read(int file, char* data, int length) {
   *        // For "standard input" (stdin)
   *        if (file == STDIN_FILENO) {
   *            return Serial_Read(data, static_cast<size_t>(length));
   *        }
   *        return -1; // Error condition
   *    }
   *
   * For our purposes, we might pick a particular fileID and redirect to the
   * appropriate callback function to get characters from the manager's queue.
   * This makes things transparent to the target and thread-safe in the manager.
   *
   * @param data
   * @param length
   */
  int serialReadCallback(char* data, int length) {
    std::lock_guard<std::mutex> lock(log_mutex);
    for (int i = 0; i < length; i++) {
      /// NOTE: check that the input queue is not empty
      //      data[i] = target_serial_in.head();
    }
    return length;
  }

  const std::vector<std::string>& getLog() {
    return target_log;
  }

  uint32_t getTicks() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.ticks;
  }

  void setPinState(int i, bool state) {
    std::lock_guard<std::mutex> lock(target_mutex);
    target.digitalWrite(i, state);
  }

  std::array<bool, 16> getPins() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.getPinState();
  }

  SensorData getSensors() {
    std::lock_guard<std::mutex> lock(target_mutex);
    return target.getSensors();
  }

  void setSensorCallback(SensorCallbackFunction cb) {
    target.setSensorCallback(cb);
  }
};
