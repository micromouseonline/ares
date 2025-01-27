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
 * This is made possible by calling the robot's updateMotion method once for every
 * tick in the delay_ms method. The robot is quite dumb and its updateMotion
 * method just updates the sensors and the motion. At the most basic level
 * the robot is assumed to behave perfectly so there is not control system
 * as such. Instead, all motion updates are executed exactly. Sensor updates
 * are performed by the robot with a callback function provided by the main
 * application, where the representation of the physical world is held.
 *
 * There is no need for the Robot to run in its own thread.
 *
 * For testing, we can add a real delay between each call to the robot's updateMotion
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
  RobotManager(Mouse& robot)
      : m_mouse(robot),
        m_robot_mutex(),
        m_serial_output_queue(2048),
        m_binary_output_queue(1024 * 1024) {
    ARES_INFO(" RM: Assign Vehicle to Mouse");
    ARES_INFO(" RM: Assign Robot Callbacks");
    m_mouse.setSerialOut([this](char c) { this->serialOutCallback(c); });
    m_mouse.setBinaryOut([this](uint8_t b) { this->binaryOutCallback(b); });
    ARES_INFO(" RM: Start Robot");
    initRobot();
    startRobotThread();
    ARES_INFO(" RM: Initialised");
  }

  ~RobotManager() {
    ARES_INFO(" RM: Destructor...")
    stopRobotThread();
    Timer timer;
    timer.wait_ms(100);
    ARES_INFO(" RM: Join the Robot Thread")
    if (m_robot_thread.joinable()) {
      m_robot_thread.join();
      ARES_INFO(" RM: Joined Robot Thread")
    }
  }

  void initRobot() {
    ARES_INFO(" RM: Initialising Robot")
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_mouse.init();
  }

  void startRobotThread() {
    ARES_INFO(" RM: Starting Robot")
    m_robot_thread = std::thread([this]() { m_mouse.run(); });
  }

  void stopRobotThread() {
    ARES_INFO(" RM: Stopping Robot")
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_mouse.stopRunning();
  }

  void pauseRobot() {
    ARES_INFO(" RM: Pausing Robot")
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_mouse.pauseRunning();
    m_paused = true;
  }

  void resumeRobot() {
    ARES_INFO(" RM: Resuming Robot");
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_mouse.resumeRunning();
    m_paused = false;
  }

  void resetRobot() {
    ARES_INFO(" RM: Resetting Robot")
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_mouse.reset();
  }

  /////////////////////////////////////////////////////
  /// Behaviour passthroughs

  void setRobotActivity(Activity activity) {
    m_mouse.setActivity(activity);
  }

  void setRobotSpeedScale(float scale) {
    m_mouse.setSpeedUp(scale);
  }

  bool isRobotEventLogDetailed() {
    return m_mouse.getEventLogDetailed();
  }

  void setRobotEventLogDetailed(bool state) {
    m_mouse.setEventLogDetailed(state);
  }

  void setRobotContinuousMode(bool state) {
    m_mouse.setContinuous(state);
  }

  /// Some getters for the behaviour. Pause the robot, get the stuff, resume

  /***
   *We have to make a copy of the maze since it is being updated all the time
   */

  void getMazeCopy() {
    Timer timer;
    bool old_pause_state = m_paused;
    // if (old_pause_state == false) {
    m_mouse.pauseRunning();
    // }
    timer.wait_ms(0);

    m_maze_map = m_mouse.getMaze();
    if (old_pause_state == false) {
      m_mouse.resumeRunning();
    }
  }

  Maze& getMap() {
    return m_maze_map;
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
    m_serial_output_queue.push(c);
  }

  /***
   *
   * The sink for serialised binary data. On the actual robot this
   * could be a Flash chip, SD card etc.
   *
   * binaryOutCallback simply places the bytes in a queue.
   *
   * For ARES, the queue could be processed into actual storage or
   * translated into text and retained for the user
   *
   * Note that the queue must be protected with a mutex to ensure
   *      the addition of characters does not interfere with the
   *      processing, by the application, of the contents of the queue.
   */
  void binaryOutCallback(const uint8_t c) {
    std::lock_guard<std::mutex> lock(m_binary_out_mutex);
    m_binary_output_queue.push(c);
  }

  /////////////////////////////////////////////////////
  /// Vehicle passthroughs
  void setVehiclePose(float x, float y, float angle) {
    ARES_INFO(" RM: Set Robot Pose {},{} {}", x, y, angle);
  }

  SensorData getVehicleSensors() {
    return m_mouse.getVehicle().getState().sensors;
  }

  VehicleState getVehicleState() {
    return m_mouse.getVehicle().getState();
  }

  uint8_t getVehicleButtons() {
    return m_mouse.getVehicle().getState().buttons;
  }

  uint8_t getVehicleLeds() {
    return m_mouse.getVehicle().getState().leds;
  }

  /////////////////////////////////////////////////////
  /// IO Processing

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
    int count = m_serial_output_queue.size();
    static std::string s;
    while (!m_serial_output_queue.empty()) {
      char c = m_serial_output_queue.head();
      s += c;
      if (c == '\0') {
        //        log.push_back(s);
        log.push(s);
        s = "";
      }
    }
    return count;
  }

 private:
  bool m_paused = false;
  Mouse& m_mouse;
  Maze m_maze_map;
  std::mutex m_robot_mutex;
  std::thread m_robot_thread;
  std::mutex m_serial_out_mutex;
  Queue<char> m_serial_output_queue;
  std::mutex m_binary_out_mutex;
  Queue<uint8_t> m_binary_output_queue;
};
