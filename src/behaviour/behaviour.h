/******************************************************************************
 * Project: mazerunner32-ares                                                 *
 * -----                                                                      *
 * Copyright 2022 - 2024 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

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
#endif
#include <iostream>
#include <thread>
#include <vector>
#include "common/pose.h"
#include "common/timer.h"
#include "cubic.h"
#include "cubic_parameters.h"
#include "maze.h"
#include "robot/robot.h"
#include "robot/sensor-data.h"
#include "trajectory.h"
#include "trapezoid.h"

class Behaviour {
 public:
  Behaviour() : m_robot(nullptr), m_running(false), m_terminate(false), m_timeStamp(0), m_reset(false) {
    //
    m_maze.initialise();
  };

  ~Behaviour() {
    stop();  //
  }

  void setRobot(Robot& robot) {
    m_robot = &robot;  //
  }

  void reset() {
    m_reset = true;
    m_act = 0;
    waitForMove();
    // m_robot->setSpeeds(0, 0);
    // m_reset = false;
    m_maze.initialise();
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

  bool doCubicTurn(float length, float angle, float velocity) {
    startCubicTurn(length, angle, velocity);
    return waitForTurn();
  }

  bool doInPlaceTurn(float distance, float v_max, float v_end, float accel) {
    startInPlaceTurn(distance, v_max, v_end, accel);
    return waitForTurn();
  }

  void test_SS90(int counts) {
    float s = 1.0;
    float v_max = 5000.0f;
    float acc = 10000.0f;
    float turn_speed = 500.0f;
    float lead_in = 70.0f;
    float lead_out = 70.0f;
    float length = 115.0f;
    doMove(5.0 * 180 - lead_in, v_max, s * turn_speed, acc);
    doCubicTurn(length, -90, turn_speed);
    for (int i = 0; i < counts - 2; i++) {
      doMove(5.0 * 180 - lead_in - lead_out, v_max, s * turn_speed, acc);
      doCubicTurn(length, -90, turn_speed);
    }
    doMove(5.0 * 180 - lead_out, v_max, 0, acc);
    doInPlaceTurn(-90, 318, 0, 50000);
    //    doMove(75, v_max, 0, acc);  // normal sensing position
  }

  void test_SS180(int counts) {
    float s = 1.0;
    float v_max = 5000.0f;
    float acc = 10000.0f;
    float turn_speed = 600.0f;
    float lead_in = 150.0f;
    float lead_out = 150.0f;
    doMove(5.0 * 180 - lead_in, v_max, s * turn_speed, acc);
    doCubicTurn(365, -180, turn_speed);
    for (int i = 0; i < counts - 2; i++) {
      doMove(5.0 * 180 - lead_in - lead_out, v_max, s * turn_speed, acc);
      doCubicTurn(365, -180, turn_speed);
    }
    doMove(5.0 * 180 - lead_out, v_max, 0, acc);
    doInPlaceTurn(-180, 318, 0, 3000);
    doMove(75, v_max, 0, acc);  // normal sensing position
  }

  void test_circuit_run(int counts) {
    float s = 1.0;
    float v_max = 5000.0f;
    float acc = 10000.0f;
    float turn_speed = 1000.0f;
    float lead_in = 118.0f;
    float lead_out = 118.0f;
    doMove(15.0 * 180 - lead_in, v_max, s * turn_speed, acc);
    doCubicTurn(195, -90, turn_speed);
    for (int i = 0; i < counts - 2; i++) {
      doMove(15.0 * 180 - lead_in - lead_out, v_max, s * turn_speed, acc);
      doCubicTurn(195, -90, turn_speed);
    }
    doMove(15.0 * 180 - lead_out, v_max, 0, acc);
    doInPlaceTurn(-90, 318, 0, 3000);
  }

  void updateMap(RobotState& state) {
    m_leftWall = state.sensor_data.lds_power > 40;
    m_frontWall = state.sensor_data.lfs_power > 20 && state.sensor_data.rfs_power > 20;
    m_rightWall = state.sensor_data.rds_power > 40;
    switch (m_heading) {
      case DIR_N:

        m_maze.update_wall_state(m_location, DIR_N, m_frontWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_E, m_rightWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_W, m_leftWall ? WALL : EXIT);
        break;
      case DIR_E:
        m_maze.update_wall_state(m_location, DIR_E, m_frontWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_S, m_rightWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_N, m_leftWall ? WALL : EXIT);
        break;
      case DIR_S:
        m_maze.update_wall_state(m_location, DIR_S, m_frontWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_W, m_rightWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_E, m_leftWall ? WALL : EXIT);
        break;
      case DIR_W:
        m_maze.update_wall_state(m_location, DIR_W, m_frontWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_N, m_rightWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_S, m_leftWall ? WALL : EXIT);
        break;
      default:
        // This is an error. We should handle it.
        break;
    }
  }

  void turnLeft() {
    float speed = m_robot->getState().velocity;
    doMove(20, speed, 700, 5000);
    doCubicTurn(114.05, 90, speed);
    doMove(20, speed, speed, 5000);
    m_heading = left_from(m_heading);
  }

  void turnRight() {
    float speed = m_robot->getState().velocity;
    doMove(20, speed, 700, 5000);
    doCubicTurn(114.05, -90, speed);
    doMove(20, speed, speed, 5000);
    m_heading = right_from(m_heading);
  }

  void turnBack() {
    float speed = m_robot->getState().velocity;
    doMove(90, speed, 0, 5000);
    doTurn(180, 900, 0, 5000);
    doMove(90, speed, speed, 5000);
    m_heading = behind_from(m_heading);
  }

  void goForward() {
    float speed = m_robot->getState().velocity;
    doMove(180, speed, speed, 5000);
  }

  void followTo(Location target) {
    /// assume we are centred in the start cell.
    m_heading = DIR_N;
    m_location = {0, 0};
    m_robot->setPose(96.0f, 96.0f - 40.0f, 90.0f);
    RobotState robot_state = m_robot->getState();
    delay_ms(500);
    updateMap(robot_state);
    startMove(90 + 40.0f, 700, 700, 5000);
    waitForMove();
    bool finished = false;
    while (!finished) {
      if (m_terminate) {
        break;
      }
      if (m_reset) {
        // m_reset = false;
        break;
      }
      m_location = m_location.neighbour(m_heading);
      RobotState robot_state = m_robot->getState();
      updateMap(robot_state);
      if (m_location == target) {
        break;
      }

      if (!m_leftWall) {
        turnLeft();
      } else if (!m_frontWall) {
        goForward();
      } else if (!m_rightWall) {
        turnRight();
      } else {
        turnBack();
      }
    }
    doMove(90, 700, 0, 3000);
  }

  uint8_t randomHeading() {
    uint8_t turnDirection;
    if (m_leftWall && m_rightWall && m_frontWall) {
      turnDirection = BEHIND;
    } else if (m_leftWall && m_rightWall) {
      turnDirection = AHEAD;
    } else if (m_leftWall && m_frontWall) {
      turnDirection = RIGHT;
    } else if (m_rightWall && m_frontWall) {
      turnDirection = LEFT;
    } else if (m_leftWall) {
      if (m_maze.has_unknown_walls(m_location.neighbour(m_heading))) {
        turnDirection = AHEAD;
      } else if (m_maze.has_unknown_walls(m_location.neighbour(right_from(m_heading)))) {
        turnDirection = RIGHT;
      } else if (rand() % 2) {
        turnDirection = RIGHT;
      } else {
        turnDirection = AHEAD;
      }
    } else if (m_rightWall) {
      if (m_maze.has_unknown_walls(m_location.neighbour(m_heading))) {
        turnDirection = AHEAD;
      } else if (m_maze.has_unknown_walls(m_location.neighbour(left_from(m_heading)))) {
        turnDirection = LEFT;
      } else if (rand() % 2) {
        turnDirection = LEFT;
      } else {
        turnDirection = AHEAD;
      }
    } else {
      if (m_maze.has_unknown_walls(m_location.neighbour(left_from(m_heading)))) {
        turnDirection = LEFT;
      } else if (m_maze.has_unknown_walls(m_location.neighbour(right_from(m_heading)))) {
        turnDirection = RIGHT;
      } else if (rand() % 2) {
        turnDirection = LEFT;
      } else {
        turnDirection = RIGHT;
      }
    }
    return turnDirection;
  }

  int manhattanDistance(Location a, Location b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
  }

  uint8_t seekHeading() {
    uint8_t turnDirection;
    if (m_leftWall && m_rightWall && m_frontWall) {
      return BEHIND;
    }
    if (m_leftWall && m_rightWall) {
      return AHEAD;
    }
    if (m_leftWall && m_frontWall) {
      return RIGHT;
    }
    if (m_rightWall && m_frontWall) {
      return LEFT;
    }
    int distance_a = manhattanDistance(m_target, m_location.neighbour(m_heading));
    int distance_l = manhattanDistance(m_target, m_location.neighbour(left_from(m_heading)));
    int distance_r = manhattanDistance(m_target, m_location.neighbour(right_from(m_heading)));
    int distance_b = manhattanDistance(m_target, m_location.neighbour(behind_from(m_heading)));

    if (m_leftWall) {
      distance_l = 255;
    }
    if (m_rightWall) {
      distance_r = 255;
    }
    if (m_frontWall) {
      distance_a = 255;
    }
    int smallest = distance_a;
    turnDirection = AHEAD;

    if (distance_l < smallest) {
      smallest = distance_l;
      turnDirection = LEFT;
    }
    if (distance_r < smallest) {
      smallest = distance_r;
      turnDirection = RIGHT;
    }
    // if (distance_b < smallest) {
    //   smallest = distance_b;
    //   turnDirection = BEHIND;
    // }

    return turnDirection;
  }

  void wanderTo(Location target) {
    /// assume we are centred in the start cell.

    bool finished = false;
    while (!finished) {
      if (m_terminate) {
        break;
      }
      if (m_reset) {
        // m_reset = false;
        break;
      }
      m_location = m_location.neighbour(m_heading);
      RobotState robot_state = m_robot->getState();
      updateMap(robot_state);
      if (m_location == target) {
        break;
      }
      uint8_t turn = randomHeading();

      if (turn == LEFT) {
        turnLeft();
      } else if (turn == AHEAD) {
        goForward();
      } else if (turn == RIGHT) {
        turnRight();
      } else {
        turnBack();
      }
    }
    doMove(90, 700, 0, 3000);
  }

  void seekTo(Location target) {
    /// assume we are centred in the start cell.

    bool finished = false;
    while (!finished) {
      if (m_terminate) {
        break;
      }
      if (m_reset) {
        // m_reset = false;
        break;
      }
      m_location = m_location.neighbour(m_heading);
      RobotState robot_state = m_robot->getState();
      updateMap(robot_state);
      if (m_location == target) {
        break;
      }

      m_maze.flood_manhattan(target);
      unsigned char newHeading = m_maze.direction_to_smallest(m_location, m_heading);
      unsigned char hdgChange = (DIR_COUNT + newHeading - m_heading) % DIR_COUNT;
      switch (hdgChange) {
        // all these finish with the robot moving and at the sensing point
        case 0:
          goForward();
          break;
        case 2:
          turnRight();
          break;
        case 4:
          turnBack();
          break;
        case 6:
          turnLeft();
          break;
      }
      //
      // uint8_t turn = seekHeading();
      //
      // if (turn == LEFT) {
      //   turnLeft();
      // } else if (turn == AHEAD) {
      //   goForward();
      // } else if (turn == RIGHT) {
      //   turnRight();
      // } else {
      //   turnBack();
      // }
    }
    doMove(90, 700, 0, 3000);
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
        case 3:
          test_circuit_run(m_iterations * 4);
          m_act = 0;
          break;
        case 4:
          followTo(Location(0, 0));
          m_act = 0;
          break;
        case 5: {
          std::cout << "wander to..." << std::endl;
          m_heading = DIR_N;
          m_location = {0, 0};
          m_target = Location(7, 7);
          m_robot->setPose(96.0f, 96.0f - 40.0f, 90.0f);

          RobotState robot_state = m_robot->getState();
          delay_ms(500);
          updateMap(robot_state);
          startMove(90 + 40.0f, 700, 700, 5000);
          waitForMove();
          wanderTo(m_target);
          if (m_reset) {
            m_act = 0;
            break;
          }
          std::cout << "wander to... turning around" << std::endl;
          if (m_frontWall) {
            if (!m_leftWall) {
              doInPlaceTurn(90, 900, 0, 5000);
              m_heading = left_from(m_heading);
            } else if (!m_rightWall) {
              doInPlaceTurn(-90, 900, 0, 5000);
              m_heading = right_from(m_heading);
            } else {
              doInPlaceTurn(180, 900, 0, 5000);
            }
          }
          std::cout << "wander to... returning" << std::endl;
          startMove(90.04, 700, 700, 5000);
          waitForMove();
          m_target = Location(0, 0);
          wanderTo(m_target);
          if (m_reset) {
            m_act = 0;
            break;
          }
          m_heading = behind_from(m_heading);
          doInPlaceTurn(180, 900, 0, 5000);
          m_heading = behind_from(m_heading);
          m_act = 0;
        }
          std::cout << "wander to... finished" << std::endl;
          break;
        case 6: {
          std::cout << "wander to..." << std::endl;
          m_heading = DIR_N;
          m_location = {0, 0};
          m_target = Location(7, 7);
          m_robot->setPose(96.0f, 96.0f - 40.0f, 90.0f);

          RobotState robot_state = m_robot->getState();
          delay_ms(500);
          updateMap(robot_state);
          startMove(90 + 40.0f, 700, 700, 5000);
          waitForMove();
          seekTo(m_target);
          if (m_reset) {
            m_act = 0;
            break;
          }
          std::cout << "wander to... turning around" << std::endl;
          if (m_frontWall) {
            if (!m_leftWall) {
              doInPlaceTurn(90, 900, 0, 5000);
              m_heading = left_from(m_heading);
            } else if (!m_rightWall) {
              doInPlaceTurn(-90, 900, 0, 5000);
              m_heading = right_from(m_heading);
            } else {
              doInPlaceTurn(180, 900, 0, 5000);
            }
          }
          std::cout << "wander to... returning" << std::endl;
          startMove(90.04, 700, 700, 5000);
          waitForMove();
          m_target = Location(0, 0);
          seekTo(m_target);
          if (m_reset) {
            m_act = 0;
            break;
          }
          m_heading = behind_from(m_heading);
          doInPlaceTurn(180, 900, 0, 5000);
          m_heading = behind_from(m_heading);
          m_act = 0;
        }
          std::cout << "wander to... finished" << std::endl;
          break;
        default:
          // do nothing
          m_act = 0;
          break;
      }
      delay_ms(10);
    }
  }

  void go(int action, int i) {
    std::lock_guard<std::mutex> lock(g_behaviour_mutex);
    m_iterations = i;
    m_reset = false;
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
      float v = m_trap_fwd.next();
      float w = 0;
      if (m_turn_trajectory != nullptr) {
        w = m_turn_trajectory->next();
      }
      if (m_robot) {
        m_robot->setSpeeds(v, w);
        m_robot->systick(m_step_time);
        RobotState state = m_robot->getState();
        m_robot->setLed(7, state.sensor_data.lfs_power > 18);
        m_robot->setLed(6, state.sensor_data.lds_power > 40);
        m_robot->setLed(5, state.sensor_data.rds_power > 40);
        m_robot->setLed(4, state.sensor_data.rfs_power > 18);
      }

      m_timeStamp++;
      ms--;
      timer.wait_us(1000);
    }
  }

  void requestTerminate() {
    m_terminate = true;
  }

  Maze m_maze;

 private:
  bool waitForMove() {
    while (!moveFinished() && !m_terminate) {
      delay_ms(1);
    }
    delay_ms(1);
    return !m_terminate;
  }

  bool waitForTurn() {
    while (!turnFinished() && !m_terminate) {
      delay_ms(1);
    }
    return !m_terminate;
  }

  void startMove(float distance, float v_max, float v_end, float accel) {
    float v_start = m_robot->getState().velocity;
    m_trap_fwd = Trapezoid(distance, v_start, v_max, v_end, accel);
    m_trap_fwd.init(Pose());
    m_trap_fwd.begin();
  }

  bool moveFinished() {
    return m_trap_fwd.isFinished();
  }

  void startTurn(float angle, float omega_Max, float omega_end, float alpha) {
    float w_start = m_robot->getState().omega;
    std::unique_ptr<Trapezoid> trapezoid = std::make_unique<Trapezoid>(angle, w_start, omega_Max, omega_end, alpha);
    m_turn_trajectory = std::move(trapezoid);
    m_turn_trajectory->init(Pose());
    m_turn_trajectory->begin();
  }

  void startCubicTurn(float length, float angle, float velocity) {
    std::unique_ptr<Cubic> cubic = std::make_unique<Cubic>(length, angle, velocity);
    m_turn_trajectory = std::move(cubic);
    m_turn_trajectory->init(Pose());
    m_turn_trajectory->begin();
  }

  void startInPlaceTurn(float angle, float omega_Max, float omega_end, float alpha) {
    float w_start = m_robot->getState().omega;
    std::unique_ptr<Trapezoid> trapezoid = std::make_unique<Trapezoid>(angle, w_start, omega_Max, omega_end, alpha);
    std::unique_ptr<Cubic> cubic = std::make_unique<Cubic>(195, -90, m_robot->getState().velocity);
    m_turn_trajectory = std::move(trapezoid);
    m_turn_trajectory->init(Pose());
    m_turn_trajectory->begin();
  }

  bool turnFinished() {
    return m_turn_trajectory->isFinished();
  }

  Robot* m_robot = nullptr;
  Direction m_heading;
  Location m_location;
  Location m_target;
  bool m_leftWall;
  bool m_frontWall;
  bool m_rightWall;
  float m_step_time = 0.001;
  std::thread m_thread;
  std::atomic<bool> m_running;
  std::atomic<bool> m_terminate;
  std::atomic<long> m_timeStamp = 0;
  bool m_reset = false;

  std::atomic<int> m_act = 0;
  std::atomic<int> m_iterations = 0;

  Trapezoid m_trap_fwd;
  std::unique_ptr<Trajectory> m_turn_trajectory = nullptr;
};

#endif  // BEHAVIOUR_H
