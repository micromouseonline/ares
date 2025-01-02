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
#include "common/logger.h"
#include "common/logmanager.h"
#include "common/pose.h"
#include "common/timer.h"
#include "cubic.h"
#include "cubic_parameters.h"
#include "maze.h"
#include "robot/robot.h"
#include "robot/sensor-data.h"
#include "trajectory.h"
#include "trapezoid.h"

enum Activity {
  ACT_NONE,
  ACT_TEST_SS90,
  ACT_TEST_SS180,
  ACT_TEST_CIRCUIT,
  ACT_TEST_FOLLOW_TO,
  ACT_TEST_SEARCH,
};

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
    m_act = ACT_NONE;
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
    std::lock_guard<std::mutex> lock(g_behaviour_mutex);
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
    ARES_INFO("Begin following to {},{}", target.x, target.y);
    /// assume we are centred in the start cell.
    m_heading = DIR_N;
    m_location = {0, 0};
    m_robot->setPose(96.0f, 96.0f - 40.0f, 90.0f);
    RobotState robot_state = m_robot->getState();
    delay_ms(500);
    updateMap(robot_state);
    startMove(90 + 40.0f, 700, 700, 5000);
    waitForMove();
    while (!m_terminate && !m_reset) {
      m_location = m_location.neighbour(m_heading);
      ARES_INFO("Entering {},{}", m_location.x, m_location.y);
      robot_state = m_robot->getState();
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
    ARES_INFO("Complete at {},{}", m_location.x, m_location.y);
    ARES_INFO("Come to a halt");
    doMove(90, 700, 0, 3000);
    ARES_INFO("Finished following");
  }

  int manhattanDistance(Location a, Location b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
  }

  bool testSearch() {
    Log::add("Searching the maze");
    m_heading = DIR_N;
    m_location = {0, 0};
    m_target = Location(7, 7);
    m_robot->setPose(96.0f, 96.0f - 40.0f, 90.0f);
    m_robot->resetTotalDistance();

    RobotState robot_state = m_robot->getState();
    delay_ms(500);
    updateMap(robot_state);
    startMove(90 + 40.0f, 700, 700, 5000);
    waitForMove();
    Log::add("Searching starts");
    searchTo(m_target);
    Log::add("Searching stops");
    // return true;
    if (m_reset) {
      return false;
    }
    if (m_frontWall) {
      Log::add("Turning around");
      if (!m_leftWall) {
        doInPlaceTurn(90, 900, 0, 5000);
        delay_ms(200);
        m_heading = left_from(m_heading);
      } else if (!m_rightWall) {
        doInPlaceTurn(-90, 900, 0, 5000);
        delay_ms(200);
        m_heading = right_from(m_heading);
      } else {
        doInPlaceTurn(180, 900, 0, 5000);
        delay_ms(200);
      }
    }
    Log::add("Begin return to start");
    startMove(90.04, 700, 700, 5000);
    waitForMove();
    m_target = Location(0, 0);
    searchTo(m_target);
    if (m_reset) {
      return false;
    }
    m_heading = behind_from(m_heading);
    doInPlaceTurn(180, 900, 0, 5000);
    m_heading = behind_from(m_heading);
    Log::add("This round complete");
    return true;
  }

  int differenceBetween(Direction a, Direction b) {
    return (a - b + DIR_COUNT) % DIR_COUNT;
  }

  void turnToHeading(Direction newHeading) {
    uint8_t turnDirection;
    turnDirection = differenceBetween(m_heading, newHeading);
    if (turnDirection == 0) {
      return;
    }
    switch (turnDirection) {
      case LEFT:
        doTurn(90, 900, 0, 5000);
        ;
        break;
      case RIGHT:
        doTurn(-90, 900, 0, 5000);
        ;
        break;
      case BEHIND:
        doTurn(-80, 900, 0, 5000);
        ;
        break;
      default:  // anything else means we are stuck
        // do nohting
        break;
    }
  }

  /***
   * search_to will cause the mouse to move to the given target cell
   * using safe, exploration speeds and turns.
   *
   * During the search, walls will be mapped but only when first seen.
   * A wall will not be changed once it has been mapped.
   *
   * It is possible for the mapping process to make the mouse think it
   * is walled in with no route to the target if walls are falsely
   * identified as present.
   *
   * On entry, the mouse will know its location and heading and
   * will begin by moving forward. The assumption is that the mouse
   * is already facing in an appropriate direction.
   *
   * Note: that it should also be possible to have this function entered
   *     with the mouse already moving and at the sensing point. It does
   *     not do that now but will be added later. All that is required is
   *     some kind of flag that does a prequel to get to the sensing point
   *     while moving. The the main loop can tak over.
   *
   * All paths will start with a straight.
   *
   * If the function is called with handstart set true, you can
   * assume that the mouse is already backed up to the wall behind.
   *
   * Otherwise, the mouse is assumed to be centrally placed in a cell
   * and may be stationary or moving.
   *
   * The walls for the current location are assumed to be correct in
   * the map since mapping is always done by looking ahead into the
   * cell that is about to be entered.
   *
   * On exit, the mouse will be centered in the target cell still
   * facing in the direction it entered that cell. This will
   * always be one of the four cardinal directions NESW
   *
   */
  bool searchTo(Location target) {
    if (m_location == target) {
      Log::add("Already at target");
      return true;
    }
    m_maze.set_mask(MASK_OPEN);
    m_maze.flood_manhattan(target);  /////////////////////////////////////////////////////////////////The flood can fail, leaving all cells with 65535
    unsigned char newHeading = m_maze.direction_to_smallest(m_location, m_heading);
    std::string msg;
    msg = fmt::format("Searching from {},{} HDG = {} to {},{}", m_location.x, m_location.y, m_heading, target.x, target.y);
    Log::add(msg);
    uint32_t ticks = g_ticks;
    //////////////////////////////////////////////////////////////////////////////////////TERMINATING CONDITION IS WRONG !
    while (!(m_location == target)) {
      if (m_terminate || m_reset) {  /// TODO: should m_terminate just set m_reset?
        ARES_ERROR("Aborted search");
        return false;
      }
      m_location = m_location.neighbour(m_heading);
      RobotState robot_state = m_robot->getState();
      updateMap(robot_state);
      msg = "";
      msg += fmt::format(" @ {:>5} ", (int)robot_state.total_distance);
      msg += fmt::format("[{:>2},{:>2}], HDG {} ", m_location.x, m_location.y, m_heading);
      if (m_location == target) {
        break;
      }
      m_maze.flood_manhattan(target);  /////////////////////////////////////////////////////////////////The flood can fail, leaving all cells with 65535
      newHeading = m_maze.direction_to_smallest(m_location, m_heading);

      unsigned char hdgChange = (DIR_COUNT + newHeading - m_heading) % DIR_COUNT;

      msg += fmt::format(">{} -> {} ", newHeading, hdgChange);
      switch (hdgChange) {
        /// all these finish with the robot moving and at the sensing point
        case 0:
          msg += " FWD01";
          goForward();
          break;
        case 2:
          msg += " SS90ER";
          turnRight();
          break;
        case 4:
          msg += " SS180";
          turnBack();
          break;
        case 6:
          msg += " SS90L";
          turnLeft();
          break;
        default:
          ARES_ERROR("UNKNOWN DIRECTION CHANGE IN SEARCH ({})", hdgChange);
          break;
      }
      if (m_event_log_detailed) {
        Log::add(msg);
      }
    }
    /// come to a halt in the cell centre
    doMove(90, 700, 0, 3000);
    Log::add(fmt::format("  - completed after {:>8} ms", g_ticks - ticks).c_str());
    return true;
  }

  void run() {
    while (m_running) {
      switch (m_act) {
        case ACT_TEST_SS90:
          test_SS90(m_iterations);
          break;
        case ACT_TEST_SS180:
          test_SS180(m_iterations);
          break;
        case ACT_TEST_CIRCUIT:
          test_circuit_run(m_iterations * 4);
          break;
        case ACT_TEST_FOLLOW_TO:
          followTo(Location(0, 0));
          break;
        case ACT_TEST_SEARCH: {
          testSearch();
        } break;
        default:  // do nothing
          break;
      }
      m_act = ACT_NONE;
      delay_ms(10);  /// make sure the regular tasks get updated
    }
  }

  void go(int action, int i = 1) {
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
      g_ticks++;
      timer.wait_us(1000 * m_speed_up);
    }
  }

  void requestTerminate() {
    m_terminate = true;
  }

  void setSpeedUp(float speed_up) {
    m_speed_up = 1.0f / speed_up;
  }

  Maze& getMaze() {
    return m_maze;
  }

  void setEventLogDetailed(bool state) {
    m_event_log_detailed = state;
  }

  bool getEventLogDetailed() {
    return m_event_log_detailed;
  }

 private:
  bool waitForMove() {
    while (!moveFinished() && !m_terminate) {
      delay_ms(1);
    }
    delay_ms(1);
    return !m_terminate;
  }

  Maze m_maze;

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
  std::atomic<bool> m_event_log_detailed = false;
  std::atomic<bool> m_running;
  std::atomic<bool> m_terminate;  /// shuts down the thread
  std::atomic<long> m_timeStamp = 0;
  volatile std::atomic<bool> m_reset = false;

  std::atomic<int> m_act = ACT_NONE;
  std::atomic<int> m_iterations = 0;
  std::atomic<float> m_speed_up = 1.0f;

  Trapezoid m_trap_fwd;
  std::unique_ptr<Trajectory> m_turn_trajectory = nullptr;
};

#endif  // BEHAVIOUR_H
