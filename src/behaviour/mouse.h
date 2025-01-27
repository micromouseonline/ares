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
#include "action-compiler.h"
#include "application/timer.h"
#include "common/core.h"
#include "common/pose.h"
#include "common/printf/printf.h"
#include "maze.h"
#include "mouse-log.h"
#include "path-finder.h"
#include "path-printer.h"
#include "trajectories/cubic.h"
#include "trajectories/cubic_parameters.h"
#include "trajectories/spinturn.h"
#include "trajectories/straight.h"
#include "trajectory.h"
#include "vehicle/vehicle.h"

enum Activity {
  ACT_NONE,
  ACT_CONTEST,
  ACT_SEARCH,
  ACT_SPEED_1,
  ACT_SPEED_2,
  ACT_SPEED_3,
  ACT_SPEED_4,
  ACT_SPEED_5,
  ACT_TEST_FOLLOW_TO,
  ACT_TEST_CIRCUIT,
  ACT_TEST_SS90E,
  ACT_TEST_SS90F,
  ACT_TEST_SS180,
  ACT_TEST_SD45,
  ACT_TEST_SD135,
  ACT_TEST_DS45,
  ACT_TEST_DS135,
  ACT_TEST_DD90,
};

class Mouse {
 public:
  // TODO: Never instantiate the mouse without a vehicle
  Mouse(Vehicle& vehicle)
      : m_vehicle(vehicle),
        m_timeStamp(0),
        m_thread_running(false),
        m_terminate(false),
        m_reset(false),
        m_paused(false),
        m_SerialOut(nullptr),
        m_BinaryOut(nullptr) {
          // BLOCK INTENTIONALLY EMPTY
        };

  ~Mouse() {
    stopRunning();  //
  }

  void setSerialOut(SerialOut out) {
    m_SerialOut = out;
  }
  void setBinaryOut(BinaryOut out) {
    m_BinaryOut = out;
  }

  void init() {
    serialPrintf(m_SerialOut, "Mouse - initialisation\n");
    m_locked = true;
    m_current_trajectory = std::make_unique<IdleTrajectory>();
    m_vehicle.reset();
    m_maze.initialise();
    m_vehicle.setPose(96, 96, 90);
    m_heading = Direction::DIR_N;
    m_location = {0, 0};
    m_target = {7, 7};
    m_paused = false;
    m_terminate = false;
    m_thread_running = true;
    m_timeStamp = 0;
    m_ticks = 0;
    m_reset = false;
    m_activity = ACT_NONE;
    m_speed_up = 1.0f;
    m_locked = false;
  }

  Vehicle& getVehicle() {
    return m_vehicle;
  }

  void reset() {
    m_reset = true;
    m_activity = ACT_NONE;
    m_vehicle.setPose(96, 96, 90);
    m_maze.initialise();
  }

  void startRunning() {
    serialPrintf(m_SerialOut, "Mouse - start running\n");
    m_thread_running = true;
  }

  void stopRunning() {
    serialPrintf(m_SerialOut, "Mouse - stop running\n");
    m_terminate = true;
    m_thread_running = false;
  }

  void pauseRunning() {
    m_paused = true;
  }

  void resumeRunning() {
    m_paused = false;
  }

  bool isRunning() {
    return m_thread_running;
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
    float speed = m_vehicle.getState().velocity;
    doMove(20, speed, 700, 5000);
    doCubicTurn(115.6, 90, speed);
    doMove(20, speed, speed, 5000);
    setHeading(left_from(getHeading()));
  }

  void turnRight() {
    float speed = m_vehicle.getState().velocity;
    doMove(20, speed, 700, 5000);
    doCubicTurn(115.6, -90, speed);
    doMove(20, speed, speed, 5000);
    setHeading(right_from(getHeading()));
  }

  void turnBack() {
    float speed = m_vehicle.getState().velocity;
    doMove(90, speed, 0, 5000);
    doInPlaceTurn(180, 400, 0, 5000);
    doMove(90, speed, speed, 5000);
    setHeading(behind_from(getHeading()));
  }

  void goForward() {
    float speed = m_vehicle.getState().velocity;
    doMove(180, speed, speed, 5000);
  }

  void followTo(Location target) {
    /// assume we are centred in the start cell.
    setHeading(DIR_N);
    setLocation({0, 0});
    m_vehicle.setPose(96.0f, 96.0f - 40.0f, 90.0f);
    VehicleState robot_state = m_vehicle.getState();
    delay_ms(500);
    updateMap(robot_state);
    doMove(90 + 40.0f, 700, 700, 5000);
    while (!m_terminate && !m_reset) {
      setLocation(getLocation().neighbour(getHeading()));
      robot_state = m_vehicle.getState();
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
    doMove(90, 700, 0, 3000);
  }

  int manhattanDistance(Location a, Location b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
  }

  bool testSearch() {
    int run_number = 1;
    while (m_first_run || m_continuous_search) {
      m_first_run = false;
      m_logger.info("\nSearch Cycle: %d", run_number);
      run_number++;
      setHeading(DIR_N);
      setLocation({0, 0});
      m_target = Location(7, 7);
      m_vehicle.reset();
      m_vehicle.setPose(96.0f, 96.0f - 40.0f, 90.0f);

      VehicleState robot_state = m_vehicle.getState();
      delay_ms(500);
      updateMap(robot_state);
      doMove(90 + 40.0f, 700, 700, 5000);
      uint32_t t = m_ticks;
      float start_distance = robot_state.total_distance;
      searchTo(m_target);
      robot_state = m_vehicle.getState();
      float end_distance = robot_state.total_distance;
      t = m_ticks - t;
      m_logger.info("Arrived: %d mm in  %d ms", (int)(end_distance - start_distance), t);
      delay_ms(2000);
      if (m_reset || m_terminate) {
        return false;
      }
      if (m_frontWall) {
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
      m_logger.info("Return to start");
      start_distance = robot_state.total_distance;
      doMove(90.0, 700, 700, 5000);
      t = m_ticks;
      m_target = Location(0, 0);
      searchTo(m_target);
      robot_state = m_vehicle.getState();
      t = m_ticks - t;
      end_distance = robot_state.total_distance;
      m_logger.info("Arrived: %d mm in  %d ms", (int)(end_distance - start_distance), t);

      if (m_reset || m_terminate) {
        return false;
      }
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

    m_maze.set_mask(MASK_OPEN);
    m_maze.flood_manhattan(target);  /////////////////////////////////////////////////////////////////The flood can fail, leaving all cells with 65535

    m_logger.info("Searching: %d,%d,%c to %d,%d Cost:%d",  //
                  m_location.x, m_location.y,              //
                  orthoDirChar[m_heading],                 //
                  target.x, target.y, m_maze.cost(m_location));
    Direction newHeading = m_maze.direction_to_smallest(m_location, m_heading);

    //////////////////////////////////////////////////////////////////////////////////////TERMINATING CONDITION IS WRONG !
    while (!(getLocation() == target)) {
      if (m_terminate || m_reset) {  /// TODO: should m_terminate just set m_reset?
        return false;
      }
      setLocation(getLocation().neighbour(getHeading()));
      VehicleState robot_state = m_vehicle.getState();
      updateMap(robot_state);
      if (getLocation() == target) {
        break;
      }
      m_maze.flood_manhattan(target);  /////////////////////////////////////////////////////////////////The flood can fail, leaving all cells with 65535
      newHeading = m_maze.direction_to_smallest(getLocation(), getHeading());
      unsigned char hdgChange = (DIR_COUNT + newHeading - getHeading()) % DIR_COUNT;
      if (m_event_log_detailed) {
        m_logger.info("%5d [%2d,%2d] %c>%c %s",                              //
                      (int)robot_state.total_distance,                       //
                      getLocation().x, getLocation().y,                      //
                      orthoDirChar[getHeading()], orthoDirChar[newHeading],  //
                      moveNames[hdgChange]);
      }

      switch (hdgChange) {
        /// all these finish with the robot moving and at the sensing point
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
        default:
          break;
      }
    }
    /// come to a halt in the cell centre
    doMove(90, 700, 0, 3000);
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
    /// setup
    m_thread_running = true;
    std::unique_ptr<IdleTrajectory> idle = std::make_unique<IdleTrajectory>();
    m_current_trajectory = std::move(idle);
    /// loop
    while (m_thread_running) {
      if (m_paused) {
        continue;
      }
      if (m_vehicle.readButton(Button::BTN_GO)) {
        while (m_vehicle.readButton(Button::BTN_GO)) {
          delay_ms(1);
        }
      }
      if (m_vehicle.readButton(Button::BTN_RESET)) {
        while (m_vehicle.readButton(Button::BTN_RESET)) {
          delay_ms(1);
        }
      }

      switch (m_activity) {
        case ACT_TEST_SS90F:
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
      m_activity = ACT_NONE;
      delay_ms(10);  /// make sure the regular tasks get updated
    }
  }

  uint32_t getTimeStamp() {
    return m_timeStamp;  //
  }

  void systick() {
    if (m_current_trajectory && !m_current_trajectory->isFinished()) {
      m_current_trajectory->update();
      float v = m_current_trajectory->getCurrentPose().getVelocity();
      float w = m_current_trajectory->getCurrentPose().getOmega();
      m_vehicle.setSpeeds(v, w);
    } else {
      if (m_current_trajectory->getType() != Trajectory::IDLE) {
        std::unique_ptr<IdleTrajectory> idle = std::make_unique<IdleTrajectory>();
        m_current_trajectory = std::move(idle);
      }
    }
    m_vehicle.updateSensors();
    m_vehicle.updateInputs();

    m_vehicle.updateMotion(m_step_time);
    VehicleState v_state = m_vehicle.getState();
    m_vehicle.setLed(7, v_state.sensors.lfs_power > 18);
    m_vehicle.setLed(6, v_state.sensors.lds_power > 40);
    m_vehicle.setLed(5, v_state.sensors.rds_power > 40);
    m_vehicle.setLed(4, v_state.sensors.rfs_power > 18);

    if (v_state.buttons & 0x02) {
      m_timeStamp = 0;
    }
    m_vehicle.setLed(1, (v_state.buttons & Button::BTN_RESET) != 0);
    m_vehicle.setLed(0, (v_state.buttons & Button::BTN_GO) != 0);

    m_timeStamp++;
    m_ticks++;
  }
  /***
   * delay_ms must be used in any busy-wait loops required by the Behaviour code.
   * For example, if you are waiting for a sensor value to drop below a threshold,
   * then use code like:
   *    while (sensorValue > threshold) {
   *      delay_ms(1);
   *    }
   *
   * delay_ms calls the robot's updateMotion method once per iteration. That is how
   * the robot motion processing gets updated and the sensors get read. If the
   * robot updateMotion is not called it will be unresponsive.
   */
  void delay_ms(int ms) {
    Timer timer;
    while (ms > 0 && !m_terminate && !m_reset) {
      if (!m_paused) {
        systick();
        ms--;
      }
      timer.wait_us(1000 * m_speed_up);  // Avoid hogging the thread
    }
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
    while (!trajectoryFinished() && !m_terminate & !m_reset) {
      delay_ms(1);
    }
    return !m_terminate;
  }

  void startMove(float distance, float v_max, float v_end, float accel) {
    float v_start = m_vehicle.getState().velocity;
    std::unique_ptr<Straight> trapezoid = std::make_unique<Straight>(distance, v_start, v_max, v_end, accel);
    m_current_trajectory = std::move(trapezoid);
    m_current_trajectory->init(Pose());
    m_current_trajectory->begin();
  }

  bool moveFinished() {
    return m_current_trajectory->isFinished();
  }

  void startTurn(float angle, float omega_Max, float omega_end, float alpha) {
    float w_start = m_vehicle.getState().angular_velocity;
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
    float w_start = m_vehicle.getState().angular_velocity;
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
  int serialPrintf(SerialOut out, const char* format, ...) {
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

  Vehicle& m_vehicle;
  Direction m_heading = Direction::DIR_N;
  Location m_location = {0, 0};
  Location m_target = {7, 7};
  bool m_leftWall = false;
  bool m_frontWall = false;
  bool m_rightWall = false;
  float m_step_time = 0.001;
  bool m_first_run = true;
  bool m_event_log_detailed = false;
  bool m_continuous_search = true;

  uint32_t m_timeStamp = 0;
  bool m_thread_running = false;
  bool m_terminate = false;
  bool m_reset = false;
  bool m_paused = false;
  bool m_locked = false;

  uint32_t m_ticks = 0;

  std::atomic<int> m_activity = ACT_NONE;
  std::atomic<int> m_iterations = 0;
  std::atomic<float> m_speed_up = 1.0f;
  MouseLog m_logger;
  std::unique_ptr<Trajectory> m_current_trajectory = std::make_unique<IdleTrajectory>();
  SerialOut m_SerialOut;
  BinaryOut m_BinaryOut;
};
