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
#include <queue>
#ifdef _WIN32
#include <windows.h>
#else
#include <chrono>
#include <thread>
#endif
#include <iostream>
#include <vector>
#include "common/timer.h"
#include "robot/robot.h"
#include "robot/sensor-data.h"

class Behaviour {
 public:
  Behaviour()
      : m_robot(nullptr), m_running(false), m_terminate(false), m_timeStamp(0) {

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
    requestTerminate();
    if (m_running) {
      m_running = false;
      if (m_thread.joinable()) {
        m_thread.join();
      }
    }
  }

  bool doMove(float distance, float v_max, float v_end, float accel) {
    m_robot->startMove(distance, v_max, v_end, accel);
    return waitForMove();
  }

  bool doTurn(float distance, float v_max, float v_end, float accel) {
    m_robot->startTurn(distance, v_max, v_end, accel);
    return waitForTurn();
  }

  void run() {
    while (m_running) {
      // do stuff
      if (m_act) {
        doMove(14.5 * 180, 500, 500, 5000);
        doTurn(-90, 318, 0, 50000);
        for (int i = 0; i < 6; i++) {
          doMove(14 * 180, 5000, 500, 5000);
          doTurn(-90, 318, 0, 50000);
        }
        doMove(14.5 * 180, 500, 0, 5000);
        doTurn(-90, 318, 0, 50000);
        m_act = false;
      }
      delay_ms(10);
    }
  }

  void makeMove() {
    m_act = true;  //
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
    Timer timer;
    while (ms > 0) {
      if (m_robot) {
        m_robot->systick(m_step_time);
      }
      m_timeStamp++;
      ms--;
      timer.wait_us(1000);
    }
  }

  void requestTerminate() {
    m_terminate = true;
  }

 private:
  bool waitForMove() {
    while (!m_robot->moveFinished() && !m_terminate) {
      delay_ms(1);
    }
    return !m_terminate;
  }

  bool waitForTurn() {
    while (!m_robot->turnFinished() && !m_terminate) {
      delay_ms(1);
    }
    return !m_terminate;
  }

  Robot* m_robot = nullptr;
  bool m_realTime = true;
  float m_step_time = 0.001;
  std::thread m_thread;
  std::atomic<bool> m_running;
  std::atomic<bool> m_terminate;
  std::atomic<long> m_timeStamp = 0;

  std::atomic<bool> m_act = false;
};

#endif  // BEHAVIOUR_H
