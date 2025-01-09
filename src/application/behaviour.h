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
#include "../../cmake-build-debug/_deps/fmt-src/include/fmt/format.h"

#include "behaviour/mouse.h"
#include "vehicle/vehicle.h"

class Behaviour {
 public:
  Behaviour(Mouse& mouse, Vehicle& vehicle) : m_mouse(mouse), m_vehicle(vehicle) {
    //
    /// the mouse and vehicle construcors should initialise their repective
    /// instances
    mouse.setVehicle(m_vehicle);
  }

  ~Behaviour() {
    stop();
    if (m_thread.joinable()) {
      m_thread.join();
    }
  }

  void start() {
    m_running = true;
    m_mouse.start();
    m_thread = std::thread(&Behaviour::run, this);
  }

  void stop() {
    m_mouse.stop();
    m_vehicle.stop();
    m_running = false;
    if (m_thread.joinable()) {
      m_thread.join();
    }
  }

  void reset() {
    m_vehicle.reset();
    m_mouse.reset();
  }

 private:
  void run() {
    m_vehicle.start();
    m_mouse.start();
    while (m_running) {
      /// do what we need to do with the mouse and vehicle
      if (m_vehicle.isRunning() && m_mouse.isRunning()) {
        m_mouse.run();
      }
    }
    m_mouse.stop();
    m_vehicle.stop();
  }
  Mouse& m_mouse;
  Vehicle& m_vehicle;
  std::thread m_thread;
  std::atomic<bool> m_running;
};
