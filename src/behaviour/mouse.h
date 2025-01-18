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

#pragma once

/// TODO: Mouse should know nothing about application
#include "application/applog-manager.h"
#include "application/timer.h"
#include "common/pose.h"
#include "common/printf/printf.h"
#include "maze.h"
#include "motion-compiler.h"
#include "mouse-log.h"
#include "path-finder.h"
#include "path-printer.h"
#include "trajectories/cubic.h"
#include "trajectories/cubic_parameters.h"
#include "trajectories/spinturn.h"
#include "trajectories/straight.h"
#include "trajectory.h"
#include "vehicle/vehicle.h"

class Mouse {
 public:
  using SerialOut = std::function<void(const char)>;
  using BinaryOut = std::function<void(const uint8_t)>;

  // TODO: Never instantiate the mouse without a vehicle
  Mouse() : m_vehicle(nullptr), m_running(false), m_terminate(false), m_timeStamp(0), m_reset(false), m_SerialOut(nullptr) {
    //
    std::unique_ptr<IdleTrajectory> idle = std::make_unique<IdleTrajectory>();
    m_current_trajectory = std::move(idle);
    m_maze.initialise();
  };

  ~Mouse() {
    stop();  //
  }

  void setVehicle(Vehicle& vehicle) {
    m_vehicle = &vehicle;  // can be freely accessed
  }

  void setSerialOut(SerialOut out) {
    m_SerialOut = out;
  }
  void setBinaryOut(BinaryOut out) {
    m_BinaryOut = out;
  }

  void reset() {
    m_reset = true;
    m_activity = ACT_NONE;
    stop();
    m_maze.initialise();
  }

  void startRunning() {
    serialPrintf(m_SerialOut, "Hey - here we are\n");
    m_running = true;
  }

  void stop() {
    requestTerminate();
    m_running = false;
  }

  bool isRunning() {
    return m_running;
  }

  void setFirstRunState(bool state) {
    m_first_run = state;
  }

  Direction getHeading() const {
    return m_heading;
  }

  void setHeading(Direction heading) {
    m_heading = heading;
  }

  Location getLocation() const {
    return m_location;
  }

  void setLocation(Location loc) {
    m_location = loc;
  }

  bool doMove(float distance, float v_max, float v_end, float accel) {
    startMove(distance, v_max, v_end, accel);
    return waitForTrajectory();
  }

  bool doTurn(float distance, float v_max, float v_end, float accel) {
    startTurn(distance, v_max, v_end, accel);
    return waitForTrajectory();
  }

  bool doCubicTurn(float length, float angle, float velocity) {
    startCubicTurn(length, angle, velocity);
    return waitForTrajectory();
  }

  bool doInPlaceTurn(float distance, float v_max, float v_end, float accel) {
    startInPlaceTurn(distance, v_max, v_end, accel);
    return waitForTrajectory();
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

  void updateMap(VehicleState& state) {
    bool leftWall, frontWall, rightWall;

    leftWall = state.sensors.lds_power > 40;
    frontWall = state.sensors.lfs_power > 20 && state.sensors.rfs_power > 20;
    rightWall = state.sensors.rds_power > 40;
    m_frontWall = frontWall;
    m_rightWall = rightWall;
    m_leftWall = leftWall;
    /// use the local values to avoid need for aditional lock
    Location here = getLocation();
    switch (getHeading()) {
      case DIR_N:
        m_maze.update_wall_state(here, DIR_N, frontWall ? WALL : EXIT);
        m_maze.update_wall_state(here, DIR_E, rightWall ? WALL : EXIT);
        m_maze.update_wall_state(here, DIR_W, leftWall ? WALL : EXIT);
        break;
      case DIR_E:
        m_maze.update_wall_state(here, DIR_E, frontWall ? WALL : EXIT);
        m_maze.update_wall_state(here, DIR_S, rightWall ? WALL : EXIT);
        m_maze.update_wall_state(here, DIR_N, leftWall ? WALL : EXIT);
        break;
      case DIR_S:
        m_maze.update_wall_state(here, DIR_S, frontWall ? WALL : EXIT);
        m_maze.update_wall_state(here, DIR_W, rightWall ? WALL : EXIT);
        m_maze.update_wall_state(here, DIR_E, leftWall ? WALL : EXIT);
        break;
      case DIR_W:
        m_maze.update_wall_state(here, DIR_W, frontWall ? WALL : EXIT);
        m_maze.update_wall_state(here, DIR_N, rightWall ? WALL : EXIT);
        m_maze.update_wall_state(here, DIR_S, leftWall ? WALL : EXIT);
        break;
      default:
        // This is an error. We should handle it.
        break;
    }
  }

  void turnLeft() {
    float speed = m_vehicle->getState().velocity;
    doMove(20, speed, 700, 5000);
    doCubicTurn(115.6, 90, speed);
    doMove(20, speed, speed, 5000);
    setHeading(left_from(getHeading()));
  }

  void turnRight() {
    float speed = m_vehicle->getState().velocity;
    doMove(20, speed, 700, 5000);
    doCubicTurn(115.6, -90, speed);
    doMove(20, speed, speed, 5000);
    setHeading(right_from(getHeading()));
  }

  void turnBack() {
    float speed = m_vehicle->getState().velocity;
    doMove(90, speed, 0, 5000);
    doInPlaceTurn(180, 400, 0, 5000);
    doMove(90, speed, speed, 5000);
    setHeading(behind_from(getHeading()));
  }

  void goForward() {
    float speed = m_vehicle->getState().velocity;
    doMove(180, speed, speed, 5000);
  }

  void followTo(Location target) {
    ARES_INFO("Begin following to {},{}", target.x, target.y);
    /// assume we are centred in the start cell.
    setHeading(DIR_N);
    setLocation({0, 0});
    m_vehicle->setPose(96.0f, 96.0f - 40.0f, 90.0f);
    VehicleState robot_state = m_vehicle->getState();
    delay_ms(500);
    updateMap(robot_state);
    doMove(90 + 40.0f, 700, 700, 5000);
    while (!m_terminate && !m_reset) {
      setLocation(getLocation().neighbour(getHeading()));
      ARES_INFO("Entering {},{}", getLocation().x, getLocation().y);
      robot_state = m_vehicle->getState();
      updateMap(robot_state);
      if (getLocation() == target) {
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
    ARES_INFO("Complete at {},{}", getLocation().x, getLocation().y);
    ARES_INFO("Come to a halt");
    doMove(90, 700, 0, 3000);
    ARES_INFO("Finished following");
  }

  int manhattanDistance(Location a, Location b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
  }

  bool testSearch() {
    while (m_first_run) {
      m_first_run = false;
      m_logger.info("Searching the maze");
      setHeading(DIR_N);
      setLocation({0, 0});
      m_target = Location(7, 7);
      m_vehicle->reset();
      m_vehicle->setPose(96.0f, 96.0f - 40.0f, 90.0f);

      VehicleState robot_state = m_vehicle->getState();
      delay_ms(500);
      updateMap(robot_state);
      doMove(90 + 40.0f, 700, 700, 5000);
      m_logger.info("Searching starts");
      searchTo(m_target);
      m_logger.info("Searching stops");
      delay_ms(2000);
      // return true;
      if (m_reset || m_terminate) {
        return false;
      }
      if (m_frontWall) {
        m_logger.info("Turning around");
        if (!m_leftWall) {
          doInPlaceTurn(90, 900, 0, 5000);
          delay_ms(200);
          setHeading(left_from(getHeading()));
        } else if (!m_rightWall) {
          doInPlaceTurn(-90, 900, 0, 5000);
          delay_ms(200);
          setHeading(right_from(getHeading()));
        } else {
          doInPlaceTurn(180, 900, 0, 5000);
          delay_ms(200);
        }
      }
      m_logger.info("Begin return to start");
      doMove(90.04, 700, 700, 5000);
      m_target = Location(0, 0);
      searchTo(m_target);
      if (m_reset || m_terminate) {
        return false;
      }
      //      doInPlaceTurn(180, 900, 0, 5000);
      //      setHeading(behind_from(getHeading()));
      m_logger.info("This round complete");
    }
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
        doInPlaceTurn(90, 900, 0, 5000);
        break;
      case RIGHT:
        doInPlaceTurn(-90, 900, 0, 5000);
        break;
      case BEHIND:
        doInPlaceTurn(-180, 900, 0, 5000);
        break;
      default:  // anything else means we are stuck
        // do nothing
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
    if (getLocation() == target) {
      m_logger.info("Already at target");
      return true;
    }
    //    std::string msg;
    //    msg = fmt::format("\n\n\nSearching from {},{} HDG = {} to {},{}", m_location.x, m_location.y, m_heading, target.x, target.y);
    m_logger.info("Searching from %d,%d HDG=%d to %d,%d", m_location.x, m_location.y, m_heading, target.x, target.y);

    m_maze.set_mask(MASK_OPEN);
    m_maze.flood_manhattan(target);  /////////////////////////////////////////////////////////////////The flood can fail, leaving all cells with 65535
                                     //    msg = fmt::format("Flooded the maze. cost = {}", m_maze.cost(m_location));
    m_logger.info("Flooded the maze. cost = %d", m_maze.cost(m_location));
    Direction newHeading = m_maze.direction_to_smallest(m_location, m_heading);
    if (newHeading != m_heading) {
      m_logger.info("Turning to face smallest neighbour at %d", newHeading);
      //      Log::add(msg);
      //      turnToHeading(newHeading);
      //      m_heading = newHeading;
    } else {
      m_logger.info("good to go...");
    }

    //////////////////////////////////////////////////////////////////////////////////////TERMINATING CONDITION IS WRONG !
    while (!(getLocation() == target)) {
      if (m_terminate || m_reset) {  /// TODO: should m_terminate just set m_reset?
        ARES_ERROR("Aborted search");
        return false;
      }
      setLocation(getLocation().neighbour(getHeading()));
      VehicleState robot_state = m_vehicle->getState();
      updateMap(robot_state);
      std::string msg;
      msg = "";
      msg += fmt::format(" @ {:>5} ", (int)robot_state.total_distance);
      msg += fmt::format("[{:>2},{:>2}], HDG {} ", getLocation().x, getLocation().y, getHeading());
      if (getLocation() == target) {
        break;
      }
      m_maze.flood_manhattan(target);  /////////////////////////////////////////////////////////////////The flood can fail, leaving all cells with 65535
      newHeading = m_maze.direction_to_smallest(getLocation(), getHeading());

      unsigned char hdgChange = (DIR_COUNT + newHeading - getHeading()) % DIR_COUNT;

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
        m_logger.info(msg.c_str());
      }
    }
    /// come to a halt in the cell centre
    doMove(90, 700, 0, 3000);
    m_logger.info(fmt::format("  - completed ").c_str());
    return true;
  }

  /**
   * Here the top-level actions of the mouse.
   * After initialisation, control should jump to this method which will
   * run as an endless loop.
   *
   * In simulation, the robot manager will reset the m_running flag in
   * order to shut down the entire program. On the traget hardware, the
   * m_running flag would never get cleared
   *
   */
  void run() {
    std::unique_ptr<IdleTrajectory> idle = std::make_unique<IdleTrajectory>();
    m_current_trajectory = std::move(idle);
    while (m_running) {
      uint8_t buttons = m_vehicle->getButtons();
      if (buttons != 0) {
        m_vehicle->clearButtons();
        //        m_activity = m_vehicle->getActivity();
        //        m_vehicle->clearActivity();
        switch (m_activity) {
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
          case ACT_SEARCH: {
            testSearch();
            delay_ms(1);
          } break;
          default:  // do nothing
            break;
        }
      }

      m_activity = ACT_NONE;
      delay_ms(10);  /// make sure the regular tasks get updated
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
    if (ms <= 0) {
      return;
    }
    Timer timer;
    while (ms > 0) {
      if (m_vehicle) {
        if (m_current_trajectory && !m_current_trajectory->isFinished()) {
          m_current_trajectory->update();
          float v = m_current_trajectory->getCurrentPose().getVelocity();
          float w = m_current_trajectory->getCurrentPose().getOmega();
          m_vehicle->setSpeeds(v, w);
        } else {
          if (m_current_trajectory->getType() != Trajectory::IDLE) {
            std::unique_ptr<IdleTrajectory> idle = std::make_unique<IdleTrajectory>();
            m_current_trajectory = std::move(idle);
          }
        }
        m_vehicle->systick(m_step_time);
        VehicleState state = m_vehicle->getState();
        m_vehicle->setLed(7, state.sensors.lfs_power > 18);
        m_vehicle->setLed(6, state.sensors.lds_power > 40);
        m_vehicle->setLed(5, state.sensors.rds_power > 40);
        m_vehicle->setLed(4, state.sensors.rfs_power > 18);
      }

      {
        m_timeStamp++;
      }
      ms--;
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

  void setContinuous(bool state) {
    m_continuous_search = state;
  }

  bool getContinuous() {
    return m_continuous_search;
  }

  int getActivity() {
    return m_activity;
  }

  void setActivity(int activity) {
    m_activity = activity;
  }

 private:
  Maze m_maze;

  bool waitForTrajectory() {
    while (!trajectoryFinished() && !m_terminate) {
      delay_ms(1);
    }
    return !m_terminate;
  }

  void startMove(float distance, float v_max, float v_end, float accel) {
    float v_start = m_vehicle->getState().velocity;
    std::unique_ptr<Straight> trapezoid = std::make_unique<Straight>(distance, v_start, v_max, v_end, accel);
    m_current_trajectory = std::move(trapezoid);
    m_current_trajectory->init(Pose());
    m_current_trajectory->begin();
  }

  bool moveFinished() {
    return m_current_trajectory->isFinished();
  }

  void startTurn(float angle, float omega_Max, float omega_end, float alpha) {
    float w_start = m_vehicle->getState().angular_velocity;
    std::unique_ptr<Straight> trapezoid = std::make_unique<Straight>(angle, w_start, omega_Max, omega_end, alpha);
    m_current_trajectory = std::move(trapezoid);
    m_current_trajectory->init(Pose());
    m_current_trajectory->begin();
  }

  void startCubicTurn(float length, float angle, float velocity) {
    std::unique_ptr<Cubic> cubic = std::make_unique<Cubic>(length, angle, velocity);
    m_current_trajectory = std::move(cubic);
    m_current_trajectory->init(Pose());
    m_current_trajectory->begin();
  }

  void startInPlaceTurn(float angle, float omega_Max, float omega_end, float alpha) {
    float w_start = m_vehicle->getState().angular_velocity;
    std::unique_ptr<Spinturn> spinturn = std::make_unique<Spinturn>(angle, w_start, omega_Max, omega_end, alpha);
    m_current_trajectory = std::move(spinturn);
    m_current_trajectory->init(Pose());
    m_current_trajectory->begin();
  }

  bool turnFinished() {
    return m_current_trajectory->isFinished();
  }

  bool trajectoryFinished() {
    return m_current_trajectory->isFinished();
  }

 private:
  /***
   * This wrapper for the vsnprintf_ function will send its
   * output, character-by-character, to the given function.
   *
   * Call this just as you would normally call snprintf().
   *
   * The specific printf vsnprintf_ function here comes from the
   * Marco Paland lightweight printf library which uses no dynamic
   * memory and is thread safe.
   *
   * Note: Output is null-terminated to ensure compatibility with
   *       string processing functions and to mark the end of transmitted
   *       data explicitly.
   *
   * @return number of characters written including the termiator
   */
  int serialPrintf(Mouse::SerialOut out, const char* format, ...) {
    if (!out) {
      return -1;  // Return error if no valid callback is provided
    }
    const int BUFFER_SIZE = 256;
    char buffer[BUFFER_SIZE];  // Adjust size as needed
    va_list args;
    va_start(args, format);
    /// remember to leave space for a terminating null
    int count = vsnprintf_(buffer, BUFFER_SIZE - 1, format, args);
    va_end(args);

    if (count > 0) {
      for (int i = 0; i < count && i < BUFFER_SIZE - 1; ++i) {
        out(buffer[i]);
      }
      out(buffer[count] = '\0');
      count++;
    }
    return count;  // Return the number of characters written
  }

  Vehicle* m_vehicle = nullptr;
  Direction m_heading;
  Location m_location;
  Location m_target;
  bool m_leftWall;
  bool m_frontWall;
  bool m_rightWall;
  float m_step_time = 0.001;
  bool m_first_run = true;
  std::thread m_thread;
  std::atomic<bool> m_event_log_detailed = false;
  std::atomic<bool> m_continuous_search = true;
  std::atomic<bool> m_running;
  std::atomic<bool> m_terminate;  /// shuts down the thread
  std::atomic<long> m_timeStamp = 0;
  volatile std::atomic<bool> m_reset = false;

  std::atomic<int> m_activity = ACT_NONE;
  std::atomic<int> m_iterations = 0;
  std::atomic<float> m_speed_up = 1.0f;
  MouseLog m_logger;
  std::unique_ptr<Trajectory> m_current_trajectory = std::unique_ptr<IdleTrajectory>();
  SerialOut m_SerialOut;
  BinaryOut m_BinaryOut;
};
