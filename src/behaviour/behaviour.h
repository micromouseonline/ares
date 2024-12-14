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
    startMove(distance, v_max, v_end, accel);
    return waitForMove();
  }

  bool doTurn(float distance, float v_max, float v_end, float accel) {
    startTurn(distance, v_max, v_end, accel);
    return waitForTurn();
  }

  void test_SS90(int counts) {
    float s = 1.0;
    float v_max = 5000.0f;
    float acc = 10000.0f;
    float omega_max = 287.0f;
    float alpha = 2866.0f;
    float turn_speed = 500.0f;
    float lead_in = 124.0f;
    float lead_out = 123.0f;
    doMove(5.0 * 180 - lead_in, v_max, s * turn_speed, acc);
    doTurn(-90, s * omega_max, 0, alpha);
    for (int i = 0; i < counts - 2; i++) {
      doMove(5.0 * 180 - lead_in - lead_out, v_max, s * turn_speed, acc);
      doTurn(-90, s * omega_max, 0, alpha);
    }
    doMove(5.0 * 180 - lead_out, v_max, 0, acc);
    doTurn(-90, 318, 0, 50000);
  }

  void test_SS180(int counts) {
    float s = 1.0;
    float v_max = 5000.0f;
    float acc = 10000.0f;
    float omega_max = 287.0f;
    float alpha = 2866.0f;
    float turn_speed = 440.0f;
    float lead_in = 124.0f;
    float lead_out = 123.0f;
    doMove(5.0 * 180 - lead_in, v_max, s * turn_speed, acc);
    doTurn(-180, s * omega_max, 0, alpha);
    for (int i = 0; i < counts - 2; i++) {
      doMove(5.0 * 180 - lead_in - lead_out, v_max, s * turn_speed, acc);
      doTurn(-180, s * omega_max, 0, alpha);
    }
    doMove(5.0 * 180 - lead_out, v_max, 0, acc);
    doTurn(-180, 318, 0, 50000);
  }

  void run() {
    while (m_running) {
      // do stuff
      switch (m_act) {
        case 1:
          test_SS90(m_iterations);
          m_act = 0;
          break;
        case 2:
          test_SS180(m_iterations);
          m_act = 0;
          break;
        default:
          // do nothing
          break;
      }
      delay_ms(10);
    }
  }

  void go(int action, int i) {
    m_iterations = i;
    m_act = action;  //
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
      m_fwdProfile.update(m_step_time);
      m_rotProfile.update(m_step_time);
      if (m_robot) {
        m_robot->setSpeeds(m_fwdProfile.getSpeed(), m_rotProfile.getSpeed());
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
    while (!moveFinished() && !m_terminate) {
      delay_ms(1);
    }
    return !m_terminate;
  }

  bool waitForTurn() {
    while (!turnFinished() && !m_terminate) {
      delay_ms(1);
    }
    return !m_terminate;
  }

  void startMove(float distance, float v_max, float v_end, float accel) {
    m_fwdProfile.start(distance, v_max, v_end, accel);
  }

  bool moveFinished() {
    return m_fwdProfile.isFinished();
  }
  void startTurn(float angle, float omega_Max, float omega_end, float alpha) {
    m_rotProfile.start(angle, omega_Max, omega_end, alpha);
  }

  bool turnFinished() {
    return m_rotProfile.isFinished();
  }

  Robot* m_robot = nullptr;
  bool m_realTime = true;
  float m_step_time = 0.001;
  std::thread m_thread;
  std::atomic<bool> m_running;
  std::atomic<bool> m_terminate;
  std::atomic<long> m_timeStamp = 0;

  std::atomic<int> m_act = 0;
  std::atomic<int> m_iterations = 0;

  Profile m_fwdProfile;
  Profile m_rotProfile;
};

#endif  // BEHAVIOUR_H
