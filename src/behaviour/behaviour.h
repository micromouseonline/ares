//
// Created by peter on 22/11/24.
//

#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

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

#include <SFML/Graphics.hpp>
#include <atomic>
#ifdef _WIN32
#include <windows.h>
#else
#include <chrono>
#include <thread>
#endif
#include <iostream>
#include <vector>
#include "robot/robot.h"
#include "robot/sensor-data.h"

class Behaviour {
 public:
  Behaviour()
      : m_robot(nullptr), m_running(false), m_timeStamp(0) {

          //
        };

  ~Behaviour() {
    stop();  //
  }

  void setRobot(Robot& robot) {
    m_robot = &robot;  //
  }

  void start() {
    if (!m_running) {
      m_running = true;
      m_thread = std::thread(&Behaviour::run, this);
    }
  }

  void stop() {
    if (m_running) {
      m_running = false;
      if (m_thread.joinable()) {
        m_thread.join();
      }
    }
  }

  void run() {
    while (m_running) {
      // do stuff
      timedDelay(10);
    }
  }

  uint32_t getTimeStamp() {
    return m_timeStamp.load();  //
  }

  /***
   * delay_ms must be used in any busy-wait loops required by the Behaviour code.
   * For example, if you are waiting for a sensor value to drop below a threshold,
   * then use code like:
   *    while (sensorValue > threshold) {
   *      delay_ms(1);
   *    }
   *
   * delay_ms calls the robot's systick method once per iteration. That is how
   * the robot motion processing gets updated and the sensors get read. If the
   * robot systick is not called it will be unresponsive.
   */
  void delay_ms(int ms) {
    if (m_realTime) {
      timedDelay(ms);
    } else {
      fastDelay(ms);
    }
  }

  void doTick() {
    if (m_robot) {
      m_robot->systick();
    }

    m_timeStamp++;
  }

  /***
   * This version of delay_ms will attempt to run in real-time. That is. there will
   * be around 1ms between iterations. Use this when driving around with the keyboard
   * or when you want to simulate in real time for a demonstration.
   *
   * Windows and Linux do timing delays differently so if we want a timed delay
   * we need to use a different technique for each
   */
  void timedDelay(int ms) {
#ifdef _WIN32
    LARGE_INTEGER frequency, start, end;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);
    while (ms > 0) {
      doTick();
      ms--;
      do {
        QueryPerformanceCounter(&end);
      } while (end.QuadPart - start.QuadPart < frequency.QuadPart / 1000);
      start = end;
    }
#else
    const auto interval = std::chrono::milliseconds(1);
    while (ms > 0) {
      doTick();
      ms--;
      auto next_time = std::chrono::steady_clock::now() + interval;
      std::this_thread::sleep_until(next_time);
    }
#endif
  }

  /***
   * This is the full-speed version of delay_ms. It will run as fast as the PC
   * permits. That turns out to be VERY fast so take care not to overdo it. Any
   * motion will send the robot off the screen before you can lok up from the
   * keyboard.
   */
  void fastDelay(int ms) {
    for (int i = 0; i < ms; i++) {
      doTick();
    }
  }

 private:
  Robot* m_robot = nullptr;
  bool m_realTime = false;
  std::thread m_thread;
  std::atomic<bool> m_running;
  std::atomic<long> m_timeStamp = 0;
};

#endif  // BEHAVIOUR_H
