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
#include <chrono>
#include <iostream>
#include <thread>
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

  void setRobot(Robot& robot) { m_robot = &robot; }

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
      delay_ms(10);
    }
  }

  uint32_t getTimeStamp() {
    return m_timeStamp.load();  //
  }

  /// TODO: Direct keyboard drive need to run in real time
  void delay_ms(int ms) {
    while (ms > 0) {
      if (m_robot) {
        m_robot->systick();
      }
      // call robot systick
      // log state
      m_timeStamp++;
      ms--;
      // do the following if we want real-time data
      auto interval = std::chrono::microseconds(1000);
      auto next_time = std::chrono::high_resolution_clock::now() + interval;
      std::this_thread::sleep_until(next_time);
    }
    //
  }

 private:
  Robot* m_robot = nullptr;
  std::thread m_thread;
  std::atomic<bool> m_running;
  std::atomic<long> m_timeStamp = 0;
};

#endif  // BEHAVIOUR_H
