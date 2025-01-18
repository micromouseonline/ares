//
// Created by peter on 09/01/25.
//

#pragma once

/**
 *
 * The behaviour class is responsible for the behavior of the physical robot.
 *
 * It runs in its own thread though not in real time. Instead, it will run as fast
 * as the PC permits and record its state at regular intervals. These are 1ms at
 * present.
 *
 * This is made possible by calling the robot's systick method once for every
 * tick in the delay_ms method. The robot is quite dumb and its systick
 * method just updates the sensors and the motion. At the most basic level
 * the robot is assumed to behave perfectly so there is not control system
 * as such. Instead, all motion updates are executed exactly. Sensor updates
 * are performed by the robot with a callback function provided by the main
 * application, where the representation of the physical world is held.
 *
 * There is no need for the Robot to run in its own thread.
 *
 * For testing, we can add a real delay between each call to the robot's systick
 * method.
 *
 *
 * For a micromouse, Behaviour includes path planning, searching, mapping,
 * and other decision-making behaviors. It can be adapted for other types of robots.
 *
 * The Behaviour class holds an instance of the Robot class and a logical
 * representation of the Maze. The physical maze is maintained and updated by
 * the main thread.
 *
 *
 * Core Features of Behaviour
 * Behavioral Logic:
 *    - Implements decision-making algorithms such as path planning, mapping, and navigation.
 *    - Reacts to sensor data (e.g., front distance, wall proximity) for obstacle avoidance or exploration.
 *
 * Robot Interaction:
 *    - Directly interacts with the Robot instance to access pose, velocity, or other state data.
 *    - the Robot should have virtual LEDs and buttons to emulate user interaction
 *
 * Environment Access:
 *    - Maintains its own map of the maze based on sensor readings obtained from the Robot
 *
 *
 */

#include <fmt/format.h>
#include <SFML/Graphics.hpp>
#include <atomic>
#include <queue>
#ifdef _WIN32
#include <windows.h>
#else
#include <chrono>
#endif

#include <iostream>
#include <thread>
#include <vector>
#include "behaviour/mouse.h"
#include "common/queue.h"
#include "vehicle/vehicle.h"

class RobotManager {
 public:
  enum class RobotState { Stopped, Running, Paused, Resetting };

  RobotManager(Mouse& mouse, Vehicle& vehicle)  //
      : m_mouse(mouse), m_vehicle(vehicle), m_run_state(RobotState::Stopped), m_output_queue(2048) {
    //
    /// the mouse and vehicle construcors should initialise their repective
    /// instances
    ARES_INFO(" RM: Assign Vehicle to Mouse");
    m_mouse.setVehicle(m_vehicle);
    ARES_INFO(" RM: Assign Serial Callback");
    m_mouse.setSerialOut([this](char c) { this->serialOutCallback(c); });
    ARES_INFO(" RM: Start Mouse");
    m_mouse.startRunning();
    if (!m_thread.joinable()) {
      ARES_INFO(" RM: Starting Robot Thread")
      m_thread = std::thread(&RobotManager::run, this);
    }
    ARES_INFO(" RM: Initialised");
  }

  ~RobotManager() {
    stop();
    if (m_thread.joinable()) {
      ARES_INFO(" RM: Stopping Robot Thread")
      m_thread.join();
      ARES_INFO(" RM: Joined Robot Thread")
    }
  }

  void start() {
    if (m_run_state == RobotState::Stopped || m_run_state == RobotState::Paused) {
      ARES_INFO(" RM: Starting Robot")
      m_run_state = RobotState::Running;
      m_running = true;
      m_mouse.startRunning();
    }
  }

  void stop() {
    ARES_INFO(" RM: Stopping Robot")
    m_run_state = RobotState::Stopped;
    m_running = false;
    m_mouse.stop();
  }

  void pause() {
    ARES_INFO(" RM: Pausing Robot")
    if (m_run_state == RobotState::Running) {
      m_run_state = RobotState::Paused;
      m_running = false;
    }
  }

  void resume() {
    ARES_INFO(" RM: Resuming Robot")
    if (m_run_state == RobotState::Paused) {
      m_run_state = RobotState::Running;
      m_running = true;
    }
  }

  void reset() {
    ARES_INFO(" RM: Resetting Robot")
    m_run_state = RobotState::Resetting;
    m_vehicle.reset();
    m_vehicle.setPose(96.0f, 96.0f, 90.0f);
    m_mouse.reset();
    m_run_state = RobotState::Stopped;
    start();
  }

  void setVehiclePose(float x, float y, float angle) {
    ARES_INFO(" RM: Set Robot Pose {},{} {}", x, y, angle);
    m_vehicle.setPose(x, y, angle);
  }

  void setActivity(int activity, int count) {
    (void)count;
    if (m_mouse.getActivity() == ACT_NONE) {
      m_mouse.setActivity(activity);
    }
  }

  /***
   * serialOutCallback simply places the characters in a queue much as you
   * might with a transmit buffer. In this demonstration code, the
   * queue is emptied on every display frame for display in the main
   * window.
   *
   * Several such callbacks might exist to simulate other devices.
   *
   * Note that the queue must be protected with a mutex to ensure
   *      the addition of characters does not interfere with the
   *      processing, by the application, of the contents of the queue.
   */
  void serialOutCallback(const char c) {
    std::lock_guard<std::mutex> lock(m_serial_out_mutex);
    m_output_queue.push(c);
  }

  /***
   * This method is called by the application. It extracts characters from
   * the a queue and adds them to a string. When the target emits a
   * string, it will ensure they are null terminated so it is possible to
   * embed CR and LF character if desired.
   * When the null is detected, the string is added to a vector of messages
   * and the string reset.
   * Note No checks are made that there is a null present.
   *      This may not be a good thing.
   */
  //  int processOutputQueue(std::vector<std::string>& log) {
  int processOutputQueue(std::queue<std::string>& log) {
    std::lock_guard<std::mutex> lock(m_serial_out_mutex);
    int count = m_output_queue.size();
    static std::string s;
    while (!m_output_queue.empty()) {
      char c = m_output_queue.head();
      s += c;
      if (c == '\0') {
        //        log.push_back(s);
        log.push(s);
        s = "";
      }
    }
    return count;
  }

  std::string getState() {
    switch (m_run_state) {
      case RobotState::Stopped:
        return "Stopped";
        break;
      case RobotState::Paused:
        return "Paused";
        break;
      case RobotState::Resetting:
        return "Resetting";
        break;
      case RobotState::Running:
        return "Running";
        break;
      default:
        return "UNKNOWN";
        break;
    }
  }

 private:
  void run() {
    ARES_INFO(" RM: Entering thread");
    while (m_running) {
      if (m_mouse.isRunning()) {
        ARES_INFO(" RM: Mouse.run");
        m_mouse.run();  // only returns when reset
      }
    }
    ARES_INFO(" RM: Leaving thread");
    m_mouse.stop();
    m_mouse.reset();
    m_vehicle.reset();
    ARES_INFO(" RM: Exited thread");
  }
  Mouse& m_mouse;
  Vehicle& m_vehicle;
  std::thread m_thread;
  std::atomic<bool> m_running;
  RobotState m_run_state;
  std::mutex m_serial_out_mutex;
  Queue<char> m_output_queue;
};
