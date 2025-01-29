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

#include "profile.h"

const float SEARCH_TURN_SPEED = 500.0f;
const float FAST_RUN_ACCELERATION = 7000.0f;
const float SEARCH_SPEED = 600;
const float SEARCH_ACCELERATION = 5000;
const float SMOOTH_TURN_SPEED = 650;
const float FAST_TURN_SPEED = 900;
const float FAST_RUN_SPEED_MAX = 5000;
const float FAST_SEARCH_SPEED = 4000;  // speed for search dashing

const float OMEGA_SPIN_TURN = 360;
const float ALPHA_SPIN_TURN = 3600;

const int LEFT_EDGE_POS = 66;
const int RIGHT_EDGE_POS = 72;

const float BACK_WALL_TO_CENTER = 39.0f;

// the position in the cell where the sensors are sampled.
const float SENSING_POSITION = 170.0;

/// the first element is the explore acceleration
/// then there are the accelerations for the four speed runs
const int RUN_SPEEDS[] = {5, 6, 8, 10, 12};

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

enum { MS_FRESH_START, MS_SEARCH_ONE, MS_SEARCH_TWO, MS_FAST_ONE, MS_FAST_TWO, MS_FAST_THREE, MS_FAST_FOUR, MS_FINISHED };

inline int g_mouse_state = MS_FRESH_START;

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
    begin();
  };

  ~Mouse() {
    stopRunning();  //
  }

  //////////////////////////////

  void set_run_state(int state) {
    g_mouse_state = state;
  }

  void log_run_state() {
    switch (g_mouse_state) {
      case MS_SEARCH_ONE:
        m_logger.info(" First search\n");
        break;
      case MS_SEARCH_TWO:
        m_logger.info(" Second search\n");
        break;
      case MS_FAST_ONE:
        m_logger.info(" Smooth run\n");
        break;
      case MS_FINISHED:
        m_logger.info(" Finished\n");
        break;
      default:
        m_logger.info(" Unknown state\n");
        break;
    }
  }

  struct MouseState {
    WallState wall_data[1024];
    Location goal{7, 7};
    int last_run = 0;
    int mouse_state = MS_FRESH_START;
  };

  void save_state_to_flash() {
    //    MouseState state;
    //    state.goal = m_maze.goal();
    //    m_maze.write_walls(state.wall_data);
    //    writeFlash(FLASH_Sector_2, (char*)&state, sizeof(state));
    //    logWarn("Saved State to FLASH\n");
  }

  void load_state_from_flash() {
    //    MouseState state;
    //    readFlash(FLASH_Sector_2, (char*)&state, sizeof(state));
    //    m_maze.read_walls(state.wall_data);
    //    m_maze.set_goal(state.goal);
    //    Mouse::instance()->set_run_state(state.mouse_state);
    //    logWarn("Loaded State from FLASH\n");
  }

  void begin() {
    //    m_shell = new Shell(Board::instance()->serial());
    //    m_shell->println("Shell ready...");
    //    m_reporter = new Reporter(usart1);
    //    m_shell->println("Reporter ready...");
    //    m_sensors = new IR_Sensors(Board::instance()->adc());
    //    leftPostTracker.setSensor(&m_sensors->lds);
    //    rightPostTracker.setSensor(&m_sensors->rds);
    //    m_maze.set_width(16);
    //    m_maze.initialise();
    m_forward = new Profile();
    m_rotation = new Profile();
    //// This uses a lambda. I don't understand lambdas
    m_vehicle.set_systick_callback([this] { systick_callback(); });
  };

  ///////////////////////////////
  /// MR32 Cubic turns
  float s_cubic_distance;
  float s_cubic_constant;
  volatile bool cubic_active;

  void cubic_turn_update() {
    if (not cubic_active) {
      return;
    }
    float position = m_forward->distance();
    float remaining = s_cubic_distance - position;
    if (remaining > 0) {
      remaining = s_cubic_distance - position;
      float t = position * remaining;
      float omega = degrees(m_forward->speed() * s_cubic_constant * t);
      m_rotation->set_speed(omega);
    } else {
      m_rotation->finish();
      cubic_active = false;
    }
  }

  float cubic_peak_acceleration(CubicTurnParameters params, float speed) {
    float length = params.length;
    float K = 6.0f * radians(params.angle) / (length * length * length);
    float p = length / 2.0f;
    float t = p * p;
    float omega = speed * K * t;
    float acc = speed * omega;
    return acc;
  }

  float cubic_calculate_speed(const CubicTurnParameters& params, float acceleration) {
    float speed = sqrtf(acceleration * 4.0f * params.length / (6.0f * fabsf(radians(params.angle))));

    return speed;
  }

  /***
   * Immediately execute a turn using a cubic spiral profile.
   * Use the current forward speed without change.
   *
   * TODO: it looks like there is some overshoot and recovery at the end of the turn.
   */
  void execute_cubic_turn(float angle, float length) {
    if (fabsf(m_forward->speed()) < 1.0) {
      return;
    }
    //    sensors()->set_steering_mode(STEERING_OFF);
    s_cubic_distance = length;
    s_cubic_constant = 6.0f * radians(angle) / (s_cubic_distance * s_cubic_distance * s_cubic_distance);
    m_forward->set_distance(0);
    m_rotation->set_state(Profile::PS_IDLE);  // allow external setting of angular velocity
    cubic_active = true;
    while (cubic_active) {
      delay_ms(1);
    }
    m_rotation->finish();
  }

  void execute_cubic_turn(int turn_index) {
    CubicTurnParameters params = cubic_params[turn_index];
    execute_cubic_turn(params.angle, params.length);
  }

  ////////////////////////////////

  void set_steering_mode(uint8_t mode) {
    (void)mode;
    //    m_sensors->set_steering_mode(mode);
  }

  void log_pose() {
    m_logger.info(" at [%2d,%2d] heading %c \n", m_location.x, m_location.y, hdg_letters[m_heading]);
  }

  int16_t lds_value() {
    return sensors().lds_power;
  }

  int16_t lfs_value() {
    return sensors().lfs_power;
  }

  int16_t rfs_value() {
    return sensors().rfs_power;
  }

  int16_t rds_value() {
    return sensors().rds_power;
  }

  int16_t lds_raw() {
    return sensors().lds_power;
  }

  int16_t lfs_raw() {
    return sensors().lfs_power;
  }

  int16_t rfs_raw() {
    return sensors().rfs_power;
  }

  int16_t rds_raw() {
    return sensors().rds_power;
  }
  float distance() {
    return m_forward->distance();
  }

  float angle() {
    return m_rotation->distance();
  }

  float velocity() {
    return m_forward->speed();
  }

  float omega() {
    return m_rotation->speed();
  }

  float offset() const {
    return m_offset;
  }

  void set_offset(float offset) {
    m_offset = offset;
  }

  void set_cell_size(float cell_size) {
    m_cell_size = cell_size;
  }

  ////////////////////////////////

  void reset_drive_system() {
    //    m_robot->reset_drive_system();
    m_vehicle.reset();
    m_forward->reset();
    m_rotation->reset();
  }
  void set_target_velocity(float velocity) {
    m_forward->set_target_speed(velocity);
  }

  void start_move(float distance, float top_speed, float final_speed, float acceleration) {
    m_forward->start(distance, top_speed, final_speed, acceleration);
  }

  bool move_finished() {
    return m_forward->is_finished();
  }

  float move_remaining() {
    return m_forward->remaining();
  }

  void move(float distance, float top_speed, float final_speed, float acceleration) {
    m_forward->move(distance, top_speed, final_speed, acceleration);
  }

  void start_turn(float distance, float top_speed, float final_speed, float acceleration) {
    m_rotation->start(distance, top_speed, final_speed, acceleration);
  }

  bool turn_finished() {
    return m_rotation->is_finished();
  }

  void turn(float distance, float top_speed, float final_speed, float acceleration) {
    m_rotation->move(distance, top_speed, final_speed, acceleration);
  }

  void set_distance(float pos) {
    m_forward->set_distance(pos);
  }

  void set_target_speed(float pos) {
    m_forward->set_target_speed(pos);
  }

  void adjust_forward_distance(float delta) {
    m_forward->adjust_distance(delta);
  }

  ////////////////////////////////
  float get_front_distance(float sen) {
    /// TODO:  very sketchy!!
    float x = 5.0e-5 * sen * sen - 0.117 * sen + 74;
    return x;
  }

  float get_front_angle(float diff) {
    /// TODO:  very sketchy!!
    float angle = diff / 30.0f + 1.0f;
    return angle;
  }

  void align_to_wall() {
    float angle = 0;
    do {
      float diff = sensors().lfs_power - sensors().rfs_power;
      angle = get_front_angle(diff);
      turn(angle, 360, 0, 3600);
      delay_ms(100);
    } while (fabsf(angle) > 1);
    float dist = 0;
    do {
      float sum = sensors().lfs_power + sensors().rfs_power;
      dist = get_front_distance(sum) - 23;
      move(dist, 200, 0, 2000);
      delay_ms(100);
    } while (fabsf(dist) > 1);
  }

  /**
   * These are examples of ways to use the motion control functions
   */

  /**
   * The robot is assumed to be moving. This call will stop at a specific
   * distance. Clearly, there must be enough distance remaining for it to
   * brake to a halt.
   *
   * The current values for speed and acceleration are used.
   *
   * Calling this with the robot stationary is undefined. Don't do that.
   *
   * @brief bring the robot to a halt at a specific distance
   */
  void stop_at(float distance) {
    float remaining = distance - m_forward->distance();
    m_forward->move(remaining, m_forward->speed(), 0, m_forward->acceleration());
  }

  /**
   * The robot is assumed to be moving. This call will stop  after a
   * specific distance has been travelled
   *
   * Clearly, there must be enough distance remaining for it to
   * brake to a halt.
   *
   * The current values for speed and acceleration are used.
   *
   * Calling this with the robot stationary is undefined. Don't do that.
   *
   * @brief bring the robot to a halt after a specific distance
   */
  void stop_after(float distance) {
    m_forward->move(distance, m_forward->speed(), 0, m_forward->acceleration());
  }

  /**
   * The robot is assumed to be moving. This utility run_function call will just
   * do a busy-wait until the forward profile gets to the supplied distance.
   *
   * @brief wait until the given distance is reached
   */
  void wait_until_distance(float distance) {
    while (m_forward->distance() < distance) {
      delay_ms(1);
    }
  }
  /**
   * The robot is assumed to be moving. This utility run_function call will just
   * do a busy-wait until the forward profile gets to the supplied distance.
   *
   * @brief wait until the given distance is reached
   */
  void wait_for_distance(float distance) {
    float d = m_forward->distance() + distance;
    while (m_forward->distance() < d) {
      delay_ms(1);
    }
  }
  ////////////////////////////////

  /**
   * Performs a turn. Regardless of whether the robot is moving or not
   *
   * The run_function is given three parameters
   *
   *  - angle  : positive is a left turn (deg)
   *  - omega  : angular velocity of middle phase (deg/s)
   *  - alpha  : angular acceleration of in/out phases (deg/s/s)
   *
   * If the robot is moving forward, it will execute a smooth, integrated
   * turn. The turn will only be repeatable if it is always performed at the
   * same forward speed.
   *
   * If the robot is stationary, it will execute an in-place spin turn.
   *
   * The parameter alpha will indirectly determine the turn radius. During
   * the accelerating phase, the angular velocity, will increase until it
   * reaches the value omega.
   * The minimum radius during the constant phase is
   *   radius = (speed/omega) * (180/PI)
   * The effective radius will be larger because it takes some time
   * for the rotation to accelerate and decelerate. The parameter alpha
   * controls that.
   *
   * Note that a real mouse may behave slightly different for left and
   * right turns and so the parameters for, say, a 90 degree left turn
   * may be slightly different to those for a 90 degree right turn.
   *
   * @brief execute an arbitrary in-place or smooth turn
   */
  void turn(float angle, float omega, float alpha) {
    // get ready to turn
    m_rotation->reset();
    m_rotation->move(angle, omega, 0, alpha);
  }

  /**
   *
   * @brief turn in place. Force forward speed to zero
   */
  void turn_in_place(float angle, float omega, float alpha) {
    m_forward->set_target_speed(0);
    while (fabsf(m_forward->speed()) > 0) {
      delay_ms(2);
    }
    turn(angle, omega, alpha);
  };

  //***************************************************************************//

  /** Search turns
   *
   * These turns assume that the robot is crossing the cell boundary but is still
   * short of the start distance of the turn.
   *
   * The turn will be a smooth, coordinated turn that should finish short of
   * the next cell boundary.
   *
   * Does NOT update the mouse heading but it possibly should
   *
   *
   * TODO: There is only just enough space to get down to turn speed. Increase turn speed?
   *
   */

  bool wait_for_turn_trigger(float threshold, float turn_point) {
    bool sensor_triggered = false;
    if (sensors().see_front_wall) {
      while (sensors().front_sum < threshold) {
        delay_ms(1);
      }
      sensor_triggered = true;
    } else {
      while (distance() < turn_point) {
        delay_ms(1);
      }
    }
    return sensor_triggered;
  }

  int get_turn_threshold() {
    int trigger = 115;
    //    if (sensors()->see_left_wall) {
    //      trigger += EXTRA_WALL_ADJUST;
    //    }
    //    if (sensors()->see_right_wall) {
    //      trigger += EXTRA_WALL_ADJUST;
    //    }
    return trigger;
  }

  void turn_SS90Ex(int turn_id) {
    int idx = turn_id - OP_TURN_SMOOTH;
    CubicTurnParameters params = cubic_params[idx];
    //    set_steering_mode(STEERING_OFF);
    set_target_speed(SEARCH_TURN_SPEED);
    float turn_point = FULL_CELL + 20;
    int threshold = get_turn_threshold();
    bool sensor_triggered = wait_for_turn_trigger(threshold, turn_point);
    char note = sensor_triggered ? 's' : 'd';
    char dir = (turn_id & 1) ? 'L' : 'R';
    m_logger.info("Turn %c %4.0f deg at D=%3.0f [%d] mm S=%4d (%c)", dir, params.angle, m_forward->distance(), (int)turn_point, sensors().front_sum, note);
    //    set_steering_mode(STEERING_OFF);
    execute_cubic_turn(idx);
    //    set_steering_mode(STEER_NORMAL);
    set_distance(HALF_CELL + params.out_offset);
  }

  ////////////////////////////////

  //***************************************************************************//
  ///////////////// Actions
  //***************************************************************************//

  void opStop() {
    m_logger.info("op_code: Stop\n");
    //    sensors()->disable();
    //    reset_drive_system();
    delay_ms(200);
  }

  void opSpinTurn(Action op) {
    m_logger.info("op_code: Spin Turn\n");
    //    set_steering_mode(STEERING_OFF);
    set_target_speed(0);
    while (fabsf(velocity()) > 0) {
      delay_ms(1);
    }
    m_rotation->reset();
    float angle = 0;
    switch (op.op_code) {
      case IP90L:
        angle = 90.0f;
        break;
      case IP90R:
        angle = -90.0f;
        break;
      case IP180L:
        angle = 180.0f;
        break;
      case IP180R:
        angle = -180.0f;
        break;
      default:
        angle = 0.0f;
        break;
    }
    turn(angle, 600, 0, 6000);
    set_distance(HALF_CELL);
  }

  /***
   * Execute a search turn Action - only for use in speed runs
   *
   * On entry the robot is expected to be travelling at the turn speed
   * approaching the threshold of the cell in which the turn will take place.
   *
   */
  void opSearchTurn(Action op) {
    int idx = op.get_smooth_turn_type();
    m_logger.info("\n%6d ", (int)m_total_distance);
    m_logger.info("%s :- ", actionNames[idx], op);
    //    int entry_dist = (int)distance();
    CubicTurnParameters params = cubic_params[idx];
    //    set_steering_mode(STEERING_OFF);
    set_target_speed(SEARCH_TURN_SPEED);
    //    int edge_dist = (int)distance();
    float turn_point = FULL_CELL - params.in_offset;
    int threshold = get_turn_threshold();
    bool sensor_triggered = wait_for_turn_trigger(threshold, turn_point);
    int sensor_value = sensors().front_sum;
    //    int trigger_pos = (int)m_total_distance;
    char note = sensor_triggered ? 's' : 'd';
    //    int turn_dist = (int)distance();
    m_logger.info("Turn @ %5d", (int)m_offset);
    m_logger.info("Trig @ %5d ", (int)m_total_distance);
    m_logger.info("SENS = %5d (%c)", sensor_value, note);
    execute_cubic_turn(idx);
    set_distance(params.out_offset);
    set_offset(params.out_offset);
  }

  /***
   * Execute a smooth turn Action - only for use in speed runs
   *
   * On entry the robot is expected to be travelling at the turn speed
   * at a position that will guarantee that it sees a falling edge
   * on the side corresponding to the turn.
   *
   * Since all distances are measured from cell centres the position
   * that the edge is detectedtells the robot where it is in relation
   * to the centre of the cell it is in.
   *
   * All turn offsets are measured from the cell centres so it should
   * be easy to have the robot wait for an edge then calulate how far
   * it has to run to get to the turn start
   *
   * TODO: if there is a SS90Ex there will be a problem
   *       because the turn will trigger on distance immediately.
   *       We need to use cell offsets.
   */
  void opSmoothTurn(Action op, bool requires_edge = true) {
    int idx = op.get_smooth_turn_type();
    m_logger.info("\n%6d ", (int)m_total_distance);
    m_logger.info("%s :- ", actionNames[idx], op);
    //    Stopwatch sw;
    CubicTurnParameters params = cubic_params[idx];
    //    set_steering_mode(STEERING_OFF);

    if (not m_first_turn) {
      //      set_target_speed(velocity());  // fix the speed at whatever we have now.
    }
    //    int entry_dist = distance();
    set_distance(0);
    //    int edge_dist = 0;
    if (requires_edge) {
      if (Action(op).is_left_turn()) {
        wait_for_edge(LEFT);
      } else {
        wait_for_edge(RIGHT);
      }
    }
    m_logger.info("Edge @ %5d ", (int)m_total_distance);
    set_distance(0);
    wait_for_distance(params.turn_offset);  // magic distance
                                            //    int turn_dist = (int)distance();
    set_distance(0);
    m_logger.info("Turn @ %5d  ", (int)m_total_distance);
    m_logger.info("V=%d mm/s", (int)velocity());
    execute_cubic_turn(idx);
    set_distance(params.out_offset);
    set_offset(params.out_offset);

    //    int exit_dist = (int)distance();
    //    uint32_t elapsed = (sw.lap() + 500) / 1000;
  }

  //===============
  void opStraight(int idx) {
    m_logger.info("\n%6d ", (int)m_total_distance);
    m_logger.info("FWD%02d  :- ", op_list[idx]);
    //    Stopwatch sw;

    float start_offset = 0;
    if (idx > 0) {
      Action last_op = Action(op_list[idx - 1]);
      int i = last_op.get_smooth_turn_type();
      CubicTurnParameters params = cubic_params[i];
      start_offset = (float)params.out_offset;
    }

    Action this_command = Action(op_list[idx]);
    int cell_count = this_command.length();

    //// TODO: FIRST TURN COMPENSATION
    //    bool immediate_turn = false;
    //    if (idx == 0 && cell_count == 1) {
    //      immediate_turn = true;
    //      // we may need to do something about an immediate first turn.
    //    }

    float end_offset = 0;
    float end_speed = SEARCH_TURN_SPEED;
    float end_distance = 0;
    Action next_command = Action(op_list[idx + 1]);
    if (next_command == OP_STOP) {
      end_offset = 0;
      end_speed = 0;
    } else if ((next_command == SS90ER) || (next_command == SS90EL)) {
      end_speed = SEARCH_TURN_SPEED;
      end_offset = HALF_CELL;
      end_distance = FULL_CELL - end_offset;
    } else {
      // all the other turns
      int i = Action(next_command).get_smooth_turn_type();
      CubicTurnParameters params = cubic_params[i];
      float speed = cubic_calculate_speed(params, m_turn_acceleration);
      // but do not exceed the maximum permitted for this turn.
      speed = std::min(speed, params.speed_max);
      end_speed = speed;
      end_offset = params.in_offset;
      end_distance = (2 * FULL_CELL - end_offset);
    }

    float straight = FULL_CELL * cell_count;
    float run_length = straight - start_offset - end_offset;
    if (m_hand_start) {
      run_length += 40;  // TODO:  measure this
      m_hand_start = false;
    }
    run_length -= 15;
    if (run_length < 5) {
      run_length = 5;
    }
    //    set_steering_mode(STEER_NORMAL);
    m_logger.info("Length = %5d ", (int)run_length);
    move(run_length, run_speed, end_speed, FAST_RUN_ACCELERATION);
    set_distance(end_distance);
    //    uint32_t elapsed = (sw.lap() + 500) / 1000;
  }
  //////////////////////////////////////////////////////////////////////////////////////
  void opDiagonal(int idx) {
    m_logger.info("\n%6d ", (int)m_total_distance);
    m_logger.info("DIA%02d  :- ", op_list[idx] - DIA0);
    //    Stopwatch sw;

    float start_offset = 0;
    if (idx > 0) {
      Action last_op = Action(op_list[idx - 1]);
      int i = last_op.get_smooth_turn_type();
      CubicTurnParameters params = cubic_params[i];
      start_offset = (float)params.out_offset;
    }

    Action this_command = Action(op_list[idx]);
    int cell_count = this_command.length();

    float end_offset = 0;
    float end_speed = SEARCH_TURN_SPEED;
    float end_distance = 0;

    Action next_command = Action(op_list[idx + 1]);
    if (next_command == OP_STOP) {
      end_offset = 0;
      end_speed = 0;
    } else {
      // all the other turns
      int i = Action(next_command).get_smooth_turn_type();
      CubicTurnParameters params = cubic_params[i];
      float speed = cubic_calculate_speed(params, m_turn_acceleration);
      // but do not exceed the maximum permitted for this turn.
      //      speed = std::min(speed, (float)params.speed_max);
      end_speed = std::min(speed, params.speed_max);
      end_offset = params.in_offset;
      end_distance = (2 * DIAG_CELL - end_offset);
    }

    float straight = DIAG_CELL * cell_count;
    float run_length = straight - start_offset - end_offset;
    run_length -= 15;
    if (run_length < 5) {
      run_length = 5;
    }
    //    set_steering_mode(STEER_DIAGONAL);
    m_logger.info("Length = %5d ", (int)run_length);
    move(run_length, run_speed, end_speed, FAST_RUN_ACCELERATION);
    set_distance(end_distance);
    //    uint32_t elapsed = (sw.lap() + 500) / 1000;
  }

  void opTurn(Action op) {
    if (op.is_spin_turn()) {
      opSpinTurn(op);
    } else if ((op == SS90ER) || (op == SS90EL)) {
      opSearchTurn(op);
    } else {
      opSmoothTurn(op);
      m_first_turn = false;
    }
    // justTurned = true;
  }

  // processes the Actions in the list until it encounters a STOP or CEND Action
  void op_execute_list(int top_speed, float acceleration) {
    uint8_t op_index = 0;
    Action action;
    run_speed = top_speed;
    m_turn_acceleration = acceleration;
    m_total_distance = 0;
    m_logger.info("\n\nBegin execution...\n");
    //    sensors()->enable();
    reset_drive_system();
    //    robot()->enable_motors();
    //    uint32_t start_time = millis();
    bool done = 0;
    m_first_turn = true;
    while (!done) {
      action = Action(op_list[op_index]);
      thisMove = action;
      if (action.op_code == ACT_END) {
        done = true;
      } else if (action.op_code == OP_STOP) {
        opStop();
        done = true;
      } else if (action.is_ortho_straight()) {
        opStraight(op_index);
      } else if (action.is_diagonal_straight()) {
        opDiagonal(op_index);
      } else if (action.is_turn_move()) {
        opTurn(action);
      } else {
        done = true;
      }
      op_index++;
    }
    //    m_logger.info("\nEnd execution after %0.1f mm\n", m_total_distance);
    //    sensors()->disable();
  }

  ////////////////////////////////

  //***************************************************************************//

  /***
   * Assumes the maze is already flooded to a single target cell and so
   * every cell will have a cost that decreases as the target is approached.
   *
   * Starting at the given cell, the algorithm repeatedly looks for the
   * smallest available neighbour and records the action taken to reach it.
   *
   * The process starts by assuming the mouse is heading NORTH in the start
   * cell since that is what would be the case at the start of a speed run.
   *
   * At each cell, the preference is to move forwards if possible
   *
   * If the pathfinder is called from any other cell, the mouse must first
   * turn to face to the smallest neighbour of that cell using the same
   * method as in this function.
   *
   * The resulting path is a simple string, null terminated, that can be
   * printed to the Serial.to make it easy to compare paths using different
   * flooding or path generating methods.
   *
   * The characters in the path string are:
   * 	'B' : always the first character, it marks the path start.
   * 	'F' : move forwards a full cell
   * 	'H' : used in speedruns to indicate movement of half a cell forwards
   * 	'R' : turn right in this cell
   * 	'L' : turn left in this cell
   * 	'A' : turn around (should never happen in a speedrun path)
   * 	'S' : the last character in the path, telling the mouse to stop
   *
   * For example, the Japan2007 maze, flooded with a simple Manhattan
   * flood, might produce the path string:
   *
   * FFFRLLRRLLRRLLRFFRRFLLFFLRFRRLLRRLLRFFFFFFFFFRFFFFFRLRLLRRLLRRFFRFFFLFFFS
   *
   * The path string is processed by the mouse directly to make it move
   * along the path. At its simplest, this is just a case of executing
   * a single movement for each character in the string, using in-place turns.
   *
   * I would strongly recommend this style of path string. Not only can the
   * strings be used to compare routes very easily, they can be printed and
   * visually compared or followed by hand.
   *
   * Path strings are easily translated into more complex paths using
   * smooth turns an they are relatively easy to turn into a set of
   * commands that will represent a diagonal path.
   *
   * Further, short path strings  can be hand-generated to test the movement
   * of the mouse or to test the setup of different turn types.
   *
   * The pathGenerator is not terribly efficient and can take up to 20ms to
   * generate a path depending on the maze, start cell and target.
   *
   */

  //
  char pathOptions[16] = {'F', 'S', 'R', 'S', 'A', 'S', 'L', 'S'};
  Direction path_make_string(Location start, Location target) {
    int cellCount = 0;
    char* pPath = path_string;
    *pPath++ = 'B';
    char command = 'F';
    Direction headingLast = m_maze.direction_to_smallest(start, DIR_N);
    Location here = start.neighbour(headingLast);
    Direction headingHere = m_maze.direction_to_smallest(here, DIR_N);
    Direction mEndHeading = headingLast;
    *pPath++ = command;
    cellCount++;
    while (here != target) {
      // stop when an unvisited cell is reached
      // actually this means it has SOME unknown walls
      if (m_maze.has_unknown_walls(here)) {
        m_logger.info("!!! Unknown walls on route at {%d,%d}\n", here.x, here.y);
      }
      unsigned char hdgChange = (DIR_COUNT + headingHere - headingLast) % DIR_COUNT;
      command = pathOptions[hdgChange];
      *pPath++ = command;
      cellCount++;
      if (command == 'R') {
        mEndHeading = right_from(mEndHeading);
      }
      if (command == 'L') {
        mEndHeading = left_from(mEndHeading);
      }
      headingLast = headingHere;
      here = here.neighbour(headingHere);
      headingHere = m_maze.direction_to_smallest(here, headingLast);
    }
    cellCount++;
    *pPath++ = 'S';
    *pPath = 0;
    return mEndHeading;
  };

  //***************************************************************************//
  /***
   * bring the mouse to a halt in the center of the current cell. That is,
   * the cell it is entering.
   */
  uint8_t stop_at_center() {
    uint8_t walls = 0;
    if (sensors().see_front_wall) {
      walls |= 1;
    };
    if (sensors().see_left_wall) {
      walls |= 2;
    };
    if (sensors().see_right_wall) {
      walls |= 4;
    };

    //    set_steering_mode(STEERING_OFF);
    float remaining = (FULL_CELL + HALF_CELL) - distance();
    m_logger.info("dead end pos = %d,  %d remaining. offset = %d\n", int(distance()), int(remaining), int(offset()));
    // finish at very low speed so we can adjust from the wall ahead if present
    stop_after(remaining);
    if ((walls & 0x01)) {
      align_to_wall();
    }
    //    start_move(remaining, m_forward->speed(), 0, 3000);
    //    if (has_wall) {
    //          while (sensors()->get_front_sum() < FRONT_REFERENCE) {
    //        delay_ms(1);
    //      }
    //    } else {
    //      while (not move_finished()) {
    //        delay_ms(1);
    //      }
    //    }
    // Be sure robot has come to a halt.
    m_vehicle.setSpeeds(0, 0);
    return walls;
  }

  ////////////////////////////////

  /**
   * The robot is already moving so it is enough to let it carry on until
   * the next sensing distance is reached.
   * Subtracting one full cell from the current distance tricks the motion
   * control into thinking it is at (or just before) the start of a new cell.
   * Then it just waits until it gets to the next sensing distance.
   */
  void move_ahead() {
    //    adjust_forward_distance(-FULL_CELL);
    int cell_count = get_run_length(m_location, m_heading);
    dash_forward(cell_count, m_cell_size);
    wait_until_distance(SENSING_POSITION);
  }

  //***************************************************************************//
  void turn_left() {
    turn_SS90Ex(SS90EL);
    set_target_speed(SEARCH_SPEED);
    wait_until_distance(SENSING_POSITION);
    m_heading = left_from(m_heading);
  }

  //***************************************************************************//
  void turn_right() {
    turn_SS90Ex(SS90ER);
    set_target_speed(SEARCH_SPEED);
    wait_until_distance(SENSING_POSITION);
    m_heading = right_from(m_heading);
  }

  //***************************************************************************//
  /***
   * As with all the search turns, this command will be called after the robot has
   * reached the search decision point and decided its next move. It is not known
   * how long that takes or what the exact distance will be.
   *
   * Turning around is always going to be an in-place operation so it is important
   * that the robot is stationary and as well centred as possible.
   *
   * It only takes 27mm of travel to come to a halt from normal search speed.
   */
  void turn_back() {
    uint8_t walls = stop_at_center();

    /// TODO: this is sooo sketchy
    if (walls & 0x01) {
      align_to_wall();
    }
    if (walls & 0x02) {  // left wall
      turn_in_place(90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
      delay_ms(50);
      align_to_wall();
      turn_in_place(90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
    } else if (walls & 0x04) {  // right wall
      turn_in_place(-90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
      delay_ms(50);
      align_to_wall();
      turn_in_place(-90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
    } else {
      turn_in_place(180, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
    }
    delay_ms(50);
    if (walls & 0x01) {
      move(-BACK_WALL_TO_CENTER, 100, 0, 1000);
      move(BACK_WALL_TO_CENTER, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    }
    set_distance(HALF_CELL);
    move(SENSING_POSITION - HALF_CELL, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    set_distance(SENSING_POSITION);
    m_heading = behind_from(m_heading);
  }
  ////////////////////////////////

  /***
   * blocking loop that waits until a side sensor finds a falling
   * edge.
   * @param side is LEFT or RIGHT
   */
  void wait_for_edge(int side) {
    //    EdgeTracker tracker;
    //    if (side == LEFT) {
    //      tracker.setSensor(&sensors()->lds);
    //    } else {
    //      tracker.setSensor(&sensors()->rds);
    //    }
    //    tracker.reset();
    //    tracker.enable();
    //    while (not tracker.edgeFound()) {
    //      tracker.update(m_total_distance);
    //      delay_ms(1);
    //    }
    //    return;
    int16_t peak = side == LEFT ? lds_value() : rds_value();
    bool looking = true;
    while (looking) {
      int16_t value = side == LEFT ? lds_value() : rds_value();
      peak = std::max(value, peak);
      if (value < peak / 2) {
        looking = false;
      }
      delay_ms(1);
    }
  }

  ////////////////////////////////
  ////////////////////////////////
  ////////////////////////////////

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
    int cells = get_run_length(m_location, m_heading);
    m_logger.info("Cell lookahead %d", cells);
    //    doMove(180, speed, speed, 5000);
    dash_forward(cells, speed);
  }

  /***
   * when exploring, the mouse looks ahead and counts the number of explored
   * cells. When that number is known, it can run those cells quickly
   * without doing any checks. The move is done as a single motion
   * profile and the mouse location gets updated accordingly.
   */
  void dash_forward(int cells, float speed) {
    if (cells <= 0) {
      return;
    }
    float distance = FULL_CELL * cells;
    float v_max = cells > 1 ? 5 * speed : speed;
    float a_max = cells > 1 ? 10000 : 5000;
    doMove(distance, v_max, speed, a_max);
    while (--cells > 0) {
      m_location = m_location.neighbour(m_heading);
    }
  }

  /***
   * During search, calculate how many visited cells lie ahead
   * maze must have been flooded
   *
   */
  int get_run_length(Location location, Direction heading) {
    int len = 1;
    // get the common error cases sorted out early
    if (not m_maze.is_exit(location, heading)) {
      return -1;
    }
    Location next_cell = location.neighbour(heading);
    if (m_maze.has_unknown_walls(location)) {
      return -2;
    }
    // now we are in with a chance of a run
    Direction new_heading = m_maze.direction_to_smallest(next_cell, heading);
    while (new_heading == heading and !m_maze.has_unknown_walls(next_cell) and next_cell != m_maze.goal()) {
      len++;
      next_cell = next_cell.neighbour(new_heading);
      new_heading = m_maze.direction_to_smallest(next_cell, new_heading);
    }
    return len;
  }

  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  /// MR32 Top level stuff

  /***
   * search_to will cause the mouse to move to the given target cell
   * using safe, exploration speeds and turns.
   *
   * During the search, walls will be mapped but only when first seen.
   * A wall will not be changed once it has been mapped.
   *
   * It is possible for the mapping process to make the mouse think it
   * is walled in with no route to the target if wals are falsely
   * identified as present.
   *
   * On entry, the mouse will know its location and heading and
   * will begin by moving forward. The assumption is that the mouse
   * is already facing in an appropriate direction.
   *
   * Note that it should also be possible to have this function entered
   * with the mouse already moving and at the sensing point. It does
   * not do that now but will be added later. All that is required is
   * some kind of flag that does a prequel to get to the sening point
   * while moving. The the main loop can tak over.
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

  bool search_to(Location target) {
    m_logger.info("\n\nSearching from {%d,%d} to: {%d,%d}\n", m_location.x, m_location.y, target.x, target.y);
    log_pose();
    m_maze.set_mask(MASK_OPEN);
    m_maze.flood_manhattan(target);
    delay_ms(200);
    //    sensors()->enable();
    reset_drive_system();
    //    robot()->enable_motors();
    //    set_steering_mode(STEERING_OFF);  // never steer from zero speed
    m_lost = false;
    if (not m_hand_start) {
      m_logger.info("Handstart\n");
      move(-BACK_WALL_TO_CENTER, SEARCH_SPEED / 4, 0, SEARCH_ACCELERATION / 2);
    }
    m_logger.info("Off we go...[%2d,%2d] : \n", target.x, target.y);
    uint32_t start_time = millis();
    //    recorderSetInterval(REC_TIME, 10);
    //    recorderReset(REC_BASIC + REC_SENSORS);
    move(BACK_WALL_TO_CENTER, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    set_distance(HALF_CELL);
    wait_until_distance(SENSING_POSITION);
    // This is where we would be if already moving on entry.
    // Each iteration of this loop starts at the sensing point
    while (m_location != target) {
      /// TODO: replace hardware call with request to ui?
      //      if (has_button_press()) {  // allow user to abort gracefully
      //        break;
      //      }
      //      set_steering_mode(STEER_NORMAL);
      m_location = m_location.neighbour(m_heading);  // the cell we are about to enter

      update_map();
      maze_has_solution();
      m_maze.flood_manhattan(target);
      if (m_location != target) {
        /// NOTE: here we can do anything to get to the next cell to be searched
        ///       For example, if there is a clear path to the target, we can run it
        ///       Or run up to the next unexplored cell on the way.
        ///       Or a clear path to the target might mean we are done searching.
        /// NOTE: The call of direction_to_smallest could be replaced with different
        ///       algorithms. For example, we might prefer to turn inwards
        ///       or prefer another smarter choice.
        /// NOTE: The algorithm chosen here is probably the only thing that mmakes search_to
        ///       different from follow_to So why not make the action an argument or otherwise
        ///       pass it to the mouse.
        unsigned char newHeading = m_maze.direction_to_smallest(m_location, m_heading);
        unsigned char hdgChange = (DIR_COUNT + newHeading - m_heading) % DIR_COUNT;
        switch (hdgChange) {
          // all these finish with the robot moving and at the sensing point
          case 0:
            move_ahead();
            break;
          case 2:
            turn_right();
            break;
          case 4:
            turn_back();
            break;
          case 6:
            turn_left();
            break;
        }
      }
      m_logger.info("\n");
    }
    // now entering the target cell so halt in the middle of that cell
    stop_at_center();

    //    sensors()->disable();
    //    set_steering_mode(STEERING_OFF);
    float elapsed = (float)(millis() - start_time) * 0.001f;
    //    float speed = robot()->m_odometry->distance() / elapsed;
    m_logger.info("\nArrived after %.3f s mm/s\n", elapsed);
    //    log_pose();
    delay_ms(250);
    save_state_to_flash();
    reset_drive_system();
    return m_lost;
  }

  /***
   * run_to should take the mouse to the target cell by whatever
   * fast means it has. There is no mapping done, just the robot()->
   *
   * It should be assumed that the maze is flooded using the CLOSED mask
   * so that the route is safe.
   *
   * run_to must calculate the path itself. This may be either by
   * pre-calculation to generate a series of operations or the path
   * may be calculated on-the-fly using the cost map from the flood.
   *
   * On entry, the mouse will know its location and heading so the
   * first operation will be to turn to face the right way for
   * the initial move. All paths will start with a straight.
   *
   * If the function is called with handstart set true, you can
   * assume that the mouse is already backed up to the wall behind.
   *
   * Like search_to, we should consider that run_to may be entered with
   * the mouse already moving across a cell boundary and that it may
   * terminate with the mouse moving at the sensing point of the cell
   * immediately before the target.
   *
   * On exit, the mouse will be centered in the target cell still
   * facing in the direction it entered that cell. This will
   * always be one of the four cardinal directions NESW
   */
  void run_to(Location target, int speed) {
    //    load_state_from_flash();
    m_logger.info("\n=========================\n");
    m_logger.info("From {%d,%d}, ", m_location.x, m_location.y);
    m_logger.info("Run to {%d,%d}\n", target.x, target.y);
    //    Board::instance()->speaker()->confirm();
    m_maze.set_mask(MASK_CLOSED);
    m_maze.flood_manhattan(target);
    Direction endHeading = path_make_string(m_location, target);

    m_logger.info((const char*)path_string);
    m_logger.info("\nCreating operations...");
    MotionCompiler::makeSmoothActions(path_string, op_list);
    //    compiler_make_orthogonal_operations(path_string, op_list);
    MotionCompiler::makeDiagonalActions(path_string, op_list, 256);
    //    compiler_make_diagonal_operations(path_string, op_list);

    printf("done:\n");
    print_action_list((Action*)op_list);
    printf("\n");
    //    sensors()->enable();
    reset_drive_system();
    //    robot()->enable_motors();
    turn_to_face(m_maze.direction_to_smallest(m_location, m_heading));
    delay_ms(200);
    printf("Running ...\n");
    m_turn_acceleration = 1000 * speed;
    m_logger.info("Speedrun: A=%d mm/s/s\n", (int)m_turn_acceleration);
    // command execution always assumes that the first
    // command starts backed up to the wall.
    //    display()->write("gyro");
    //    robot()->imu_calibrate(500);
    uint32_t start_time = millis();
    //    recorderSetInterval(REC_TIME, 10);  // update blackbox every5 mm
    //    recorderReset(REC_BASIC + REC_SENSORS);
    //    display()->write("go..");
    // NOTE: acc =7500 seems to have no over/undershoot
    m_total_distance = 0;
    op_execute_list(FAST_RUN_SPEED_MAX, m_turn_acceleration);
    delay_ms(250);

    //    display()->write("done");
    //    recorderDisable();
    //    m_sensors->enable();
    m_logger.info("Finish : %d\n", sensors().front_sum);
    if (sensors().see_front_wall) {
      //      robot()->enable_motors();
      while (sensors().front_sum > 900) {
        move(-2, 100, 0, 1000);
      }
      if (sensors().front_sum > 400) {
      }
      align_to_wall();
    }
    m_heading = endHeading;
    m_location = target;
    float elapsed = (float)(millis() - start_time) * 0.001f;
    float avg_speed = m_total_distance / elapsed;
    m_logger.info("\nArrived after %d mm in %.3f s => %6.1f mm/s\n", (int)m_total_distance, elapsed, avg_speed);
    //    log_pose();
    //    m_logger.info("\nRun finished LOC = [%d,%d], HDG = %c\n", m_location.x, m_location.y, hdg_letters[m_heading]);
  }

  void turn_to_face(Direction newHeading) {
    unsigned char hdgChange = (newHeading + DIR_COUNT - m_heading) % DIR_COUNT;
    //    m_logger.info("turn to face %c from %c\n", hdg_letters[newHeading], hdg_letters[m_heading]);
    //    robot()->enable_motors();
    switch (hdgChange) {
      case 0:
        break;
      case 2:
        turn_in_place(-90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
        break;
      case 4:
        turn_in_place(180, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
        break;
      case 6:
        turn_in_place(90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
        break;
      default:
        // do nothing;
        break;
    }
    m_heading = newHeading;
    reset_drive_system();
    delay_ms(100);
  }

  void update_map() {
    bool leftWall = sensors().see_left_wall;
    bool frontWall = sensors().see_front_wall;
    bool rightWall = sensors().see_right_wall;
    char w[] = "--- ";
    if (leftWall) {
      w[0] = 'L';
    }
    if (frontWall) {
      w[1] = 'F';
    }
    if (rightWall) {
      w[2] = 'R';
    }
    //    m_logger.info("%6d - @%3d {%d,%d} %c %s", (int)m_robot->m_odometry->distance(), (int)m_forward->distance(), m_location.x, m_location.y,
    //    hdg_letters[m_heading],
    //            w);
    switch (m_heading) {
      case DIR_N:
        m_maze.update_wall_state(m_location, DIR_N, frontWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_E, rightWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_W, leftWall ? WALL : EXIT);
        break;
      case DIR_E:
        m_maze.update_wall_state(m_location, DIR_E, frontWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_S, rightWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_N, leftWall ? WALL : EXIT);
        break;
      case DIR_S:
        m_maze.update_wall_state(m_location, DIR_S, frontWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_W, rightWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_E, leftWall ? WALL : EXIT);
        break;
      case DIR_W:
        m_maze.update_wall_state(m_location, DIR_W, frontWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_N, rightWall ? WALL : EXIT);
        m_maze.update_wall_state(m_location, DIR_S, leftWall ? WALL : EXIT);
        break;
      default:
        // This is an error. We should handle it.
        break;
    }
  }

  bool maze_has_solution() {
    m_maze.set_mask(MASK_CLOSED);
    int cost_closed = m_maze.flood_manhattan(m_maze.goal());
    m_maze.set_mask(MASK_OPEN);
    int cost_open = m_maze.flood_manhattan(m_maze.goal());
    bool solved = (cost_open == cost_closed);

    if (solved) {
      //      Board::instance()->leds()->on(5);
      if (cost_open == UINT16_MAX) {
        //        Board::instance()->leds()->set_pattern(0xff);
        m_lost = true;
      }
    } else {
      //      Board::instance()->leds()->off(5);
    }
    return solved;
  }

  void log_state() {
    //    bool echo = logEchoState();
    m_logger.info("Mouse state is: ");
    switch (g_mouse_state) {
      case MS_SEARCH_ONE:
        m_logger.info(" First search");
        break;
      case MS_SEARCH_TWO:
        m_logger.info(" Second search");
        break;
      case MS_FAST_ONE:
        m_logger.info(" Smooth run");
        break;
      case MS_FINISHED:
        m_logger.info(" Finished");
        break;
      default:
        m_logger.info(" Unknown state");
        break;
    }
    //    logSetEcho(echo);
  }
  /***
   * The mouse is expected to be in the start cell heading NORTH
   * The maze may, or may not, have been searched.
   * There may, or may not, be a solution.
   *
   * This simple searcher will just search to goal, turn around and
   * search back to the start. At that point there will be a route
   * but it is unlikely to be optimal.
   *
   * the mouse can run this route by creating a path that does not
   * pass through unvisited cells.
   *
   * A better searcher will continue until a path generated through all
   * cells, regardless of visited state, does not pass through any
   * unvisited cells.
   *
   * The return value is not currently used but could indicate whether
   * the maze is 'solved'. That is, whether there is any need to search
   * further.
   *
   */
  int search_maze() {
    m_logger.info("\n\n======================\nSearch Maze\n");
    if (maze_has_solution() == true) {
      return 0;
    }
    m_hand_start = true;
    m_location = START;
    m_heading = DIR_N;
    bool lost = search_to(m_maze.goal());
    if (lost) {
      //      robot()->reset_drive_system();
      m_vehicle.reset();
      return -1;
    }
    if (maze_has_solution()) {
      //      Board::instance()->speaker()->complete();
    } else {
      //      Board::instance()->speaker()->good();
    }
    /// and now back to the start
    m_maze.set_mask(MASK_OPEN);
    m_maze.flood_manhattan(START);
    Direction best_direction = m_maze.direction_to_smallest(m_location, m_heading);
    turn_to_face(best_direction);
    m_hand_start = false;
    lost = search_to(START);
    if (lost) {
      //      robot()->reset_drive_system();
      m_vehicle.reset();
      return -1;
    }
    //    robot()->enable_motors();
    //    m_sensors->enable();
    align_to_wall();
    turn_in_place(90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
    delay_ms(100);
    align_to_wall();
    turn_in_place(90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
    delay_ms(100);
    setHeading(DIR_N);
    //    robot()->enable_motors();
    //    set_steering_mode(STEERING_OFF);  // never steer from zero speed
    move(-BACK_WALL_TO_CENTER - 40, 100, 0, 1000);
    m_hand_start = true;
    //    robot()->stop();

    //    robot()->reset_drive_system();
    m_vehicle.reset();

    if (maze_has_solution()) {
      //      Board::instance()->speaker()->success();
    }
    /// show what we have discovered
    m_maze.set_mask(MASK_CLOSED);
    m_maze.flood_manhattan(m_maze.goal());
    path_make_string({0, 0}, m_maze.goal());
    m_logger.info("%s\n", path_string);
    printf("%s\n", path_string);
    MotionCompiler::makeSmoothActions(path_string, op_list);
    //    compiler_make_orthogonal_operations(path_string, op_list);
    //    print_action_list(op_list);
    return 0;
  }

  /***
   * The mouse is expected to be in the start cell heading NORTH
   * The maze must have a solution.
   *
   * The mouse will perform a speed run to the goal and search back to the start
   *
   */
  int run_maze(int speed) {
    m_logger.info("\n\n======================\nRun Maze\n");

    run_to(m_maze.goal(), speed);
    /// and now back to the start
    m_logger.info("done run\n");
    m_maze.set_mask(MASK_OPEN);
    m_maze.flood_manhattan(START);
    Direction best_direction = m_maze.direction_to_smallest(m_location, m_heading);
    m_logger.info("turning\n");
    turn_to_face(best_direction);
    m_logger.info("turned\n");
    m_hand_start = false;
    // tacky but - don't wait to get back to the start before advancing the run count;
    g_mouse_state += 1;
    g_mouse_state %= MS_FINISHED;

    search_to(START);
    /// get ready for the next sequence
    //    m_sensors->enable();
    reset_drive_system();
    //    robot()->enable_motors();
    align_to_wall();
    turn_in_place(90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
    delay_ms(100);
    align_to_wall();
    turn_in_place(90, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
    delay_ms(100);
    setHeading(DIR_N);
    //    set_steering_mode(STEERING_OFF);  // never steer from zero speed
    move(-BACK_WALL_TO_CENTER - 40, 100, 0, 1000);
    m_hand_start = true;

    //    robot()->stop();
    //    robot()->reset_drive_system();
    m_vehicle.reset();
    return 0;
  }

  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////

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

  uint32_t millis() {
    return m_timeStamp;
  }

  ////////////////////////////////
  /***
   * The systick_callback is called by the vehicle at every systick.
   * It provides a way to have the Mouse update the profilers so that
   * ownership of the current profile stays with the mouse, not the robot.
   */
  void systick_callback() {
    // at the very least, this is where the mouse gets to update the desired velocity.
    //    m_sensors->update();
    //    m_robot->set_steering_feedback(m_sensors->get_steering_feedback());
    m_forward->update();
    m_offset = m_offset + m_forward->increment();
    m_total_distance = m_total_distance + m_forward->increment();
    if (m_offset > m_cell_size) {
      m_offset = m_offset - m_cell_size;
    }
    m_rotation->update();
    cubic_turn_update();
    m_vehicle.setSpeeds(m_forward->speed(), m_rotation->speed());
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
        m_vehicle.systick();
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

  float getOffset() {
    return m_offset;
  }

  void setOffset(float offset) {
    m_offset = offset;
  }

  float getDistance() {
    return m_total_distance;
  }

  void setDistance(float distance) {
    m_total_distance = distance;
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

  SensorData sensors() {
    ATOMIC
    return m_vehicle.getState().sensors;
  }

  /// From MR32 mostly

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

  //// from MR32 - more or less
  float m_total_distance = 0;
  float m_offset = 0;  /// distance through one cell
  float m_cell_size = 180.0f;
  Direction m_heading = Direction::DIR_N;
  Location m_location = {0, 0};
  float m_turn_acceleration = 8.0f;
  bool m_first_turn = false;
  bool m_lost = false;
  float run_speed;
  bool m_hand_start = false;
  Action lastMove;
  Action thisMove;
  Action nextMove;
  Profile* m_forward;
  Profile* m_rotation;
  char path_string[2048];
  uint8_t op_list[2048];
};
