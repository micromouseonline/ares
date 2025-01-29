// robot.h
// Created by peter on 22/11/24.
// Defines the Robot class which models the physical behaviour of the robot
//
#pragma once
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include "common/core.h"
#include "common/singleton.h"

/**
 * @brief The Vehicle class models the physical robot's behavior and movement.
 *
 * The Vehicle class acts as a simulated vehicle that moves and turns on the screen,
 * mimicking the dynamic behavior of a real-world robot. It is designed to be
 * generic and independent of any specific behavior or control logic, except for
 * non-holonomic constraints and sensor data collection.
 *
 * ### Key Features:
 * - **Movement and State:**
 *   The robot's state (position, orientation, and velocity) is maintained in a
 *   single struct. The forward and angular velocities are the primary inputs,
 *   and the robot assumes ideal instantaneous matching of these values.
 *
 * - **Sensor Data:**
 *   Sensor readings are evaluated via a callback function provided by the
 *   application. The Robot stores the sensor data as raw values (0-1023), while
 *   interpreting these values is left to the Behavior layer. This design mimics
 *   real-world sensor operation where the robot queries the environment but
 *   delegates interpretation to higher-level logic.
 *
 * - **Virtual inputs and monitoring:**
 *   When the Vehicle requests sensor data it sends a copy of its current state
 *   to the application. That way the application can monitor the vehicle without
 *   having to call any of its methods. For example, the state of the indicator
 *   LEDS is read to updatethe application UI.
 *
 *   The application sends back the sensor data as part of another complete copy
 *   of the vehicle state. The main reason is to allow the application to perform
 *   tasks like simulate the pressing of buttons on the vehicle or update the
 *   settings switches or their equivalent
 *
 * - **Simulation of Hardware Interrupts:**
 *   The `updateMotion()` method simulates a hardware timer interrupt, typically running
 *   at 1kHz on real robots. It updates low-level controllers, monitors encoders
 *   and sensors, and handles motion processing. This ensures accurate time
 *   synchronization and hardware-like behavior.
 *
 * - **Time Management:**
 *   The Robot maintains a millisecond-accurate counter (`m_state.ticks`), which serves
 *   as a timestamp for logging and timing purposes.
 *
 * ### Relationship with Application and Behavior:
 * - The **Application** holds the physical world data and provides the sensor
 *   callback, determining the robot's relationship with the environment.
 * - The **Behavior** layer interacts with the Robot to control its movement,
 *   interpret sensor data, and define higher-level actions.
 *
 * ### Multi-threaded running
 *   The behaviour and the vehicle code run in a shared thread
 *   Thus the behaviour code (the muse) is able to call any vehicle method
 *   without having to worry about shared data and mutexes
 *   The Robot manager and the application both run in a different thread
 *   and so should not call any vehicle methods while the vehicle is running
 *
 * This separation of concerns ensures modularity, where the Vehicle focuses solely
 * on physical behavior, and the Behavior or Application layers handle interpretation
 * and control logic.
 */

enum Button {
  BTN_GO = (1 << 0),
  BTN_RESET = (1 << 1),
};

enum Led {
  LED_1 = (1 << 0),
  LED_2 = (1 << 1),
  LED_3 = (1 << 2),
  LED_4 = (1 << 3),
};

struct SensorData {
  float lfs_distance = 0;
  float lds_distance = 0;
  float rds_distance = 0;
  float rfs_distance = 0;
  float lfs_power = 0;
  float lds_power = 0;
  float rds_power = 0;
  float rfs_power = 0;
  float front_sum = 0;
  float front_diff = 0;
  bool see_front_wall = false;
  bool see_left_wall = false;
  bool see_right_wall = false;
};

struct VehicleInputs {
  SensorData sensors;
  uint8_t buttons = 0;
};
/**
 * Positions here are in world coordinates with
 * the origin in the bottom left, x-axis to the right
 * Angles are with respect to the x-axis. Positive angles
 * are anti-clockwise.
 *
 */
struct VehicleState {
  uint32_t ticks = 0;
  float x = 0;
  float y = 0;
  float angle = 0;
  float velocity = 0;
  float angular_velocity = 0;
  float total_distance = 0;  // accumulated from last reset
  uint8_t leds = 0;          // bitfield for led states
  uint8_t buttons = 0;       // bitfield for button states
  SensorData sensors;
};

struct Velocities {
  float velocity = 0;
  float omega = 0;
};

struct MotorVoltages {
  float left = 0;
  float right = 0;
};

/// Returns a SensorData struct
using SensorDataCallback = std::function<VehicleInputs(VehicleState)>;

class Vehicle {
 public:
  Vehicle()
      : m_state() {
    reset();
  }

  ~Vehicle() {
    // BLOCK INTENTIONALLY EMPTY
  }

  void reset() {
    m_state.ticks = 0;
    m_state.total_distance = 0;
    setSpeeds(0, 0);
  }

  void begin() {
    m_state.ticks = 0;
    m_state.total_distance = 0;
    setSpeeds(0, 0);
    m_initialised = true;
  }

  /***
   * In the hardware, systick is a timer interrupt.
   * In the simulation, this is called from the behaviour's delay_ms() method
   * to advance the state of the vehicle
   * You could call it asynchronously using another thread but that just
   * makes things more complicated.
   */
  void systick() {
    //    boardUpdate();  /// update the state of the board
    ///   speaker
    ///   buttons
    ///   display
    ///   battery

    updateSensors();  /// turn adc data into useful sensor results
    updateMotion(m_step_time);

    if (systick_mouse_callback) {
      systick_mouse_callback();
    }
    Velocities actual_velocities;
    float steering_feedback = get_steering_feedback();
    MotorVoltages motor_voltages;
    motor_voltages = motorControllersUpdate(desired_velocities, actual_velocities, steering_feedback);  /// calculate required motor output voltages
    set_motor_voltage(motor_voltages.left, motor_voltages.right);                                       /// set the output voltage
    updateLeds();
    //    recorderUpdate(); /// process the next line of blackbox data

    /// calculate time taken in this method
  }

  void set_steering_feedback(float steering_fb) {
    m_steering_fb = steering_fb;
  }
  float get_steering_feedback() {
    return m_steering_fb;
  }

  void set_systick_callback(SystickMouseCallback callback) {
    systick_mouse_callback = callback;
  }

  ///// stubs for sim //////////////////////////////////////////
  void updateLeds() {
    setLed(7, m_state.sensors.lfs_power > 18);
    setLed(6, m_state.sensors.lds_power > 40);
    setLed(5, m_state.sensors.rds_power > 40);
    setLed(4, m_state.sensors.rfs_power > 18);
    setLed(1, (m_state.buttons & Button::BTN_RESET) != 0);
    setLed(0, (m_state.buttons & Button::BTN_GO) != 0);
  }
  void reset_imu(int ms) {
    //    Board::instance()->gyro()->reset(ms);
  }

  void reset_drive_system() {
    //    m_motors->controller_disable();
    //    m_odometry->reset();
    //    m_motors->reset();
    setSpeeds(0, 0);
    //    m_pwm->stop();
  }

  MotorVoltages motorControllersUpdate(Velocities desired, Velocities actual, float steering_feedback) {
    return {0, 0};
  }

  void enable_motors() {
    //      m_motors->controller_enable();
  }

  void stop() {
    //      m_motors->reset();
    //      m_pwm->stop();  // TODO: is this redundant
  }

  void imu_calibrate(int samples = 500) {
    //      Board::instance()->gyro()->reset(samples);
  }

  void set_motor_voltage(float left, float right) {
    //    m_pwm->set_volts(left, right, Board::instance()->battery()->voltage());
  }

  float battery_voltage() {
    return 7.4f;
    //    return Board::instance()->battery()->voltage();
  }

  uint16_t system_load() {
    return 1000;
  }

  /***
   * while testing this normally tells me the battery is going.
   * @param message
   */
  bool has_panic() {
    return m_has_panic;
  }

  /***
   * In the hardware, this would not return.
   * Instead,
   *    the LEDS would all flash,
   *    a message appears on the display
   *    the speaker beeps
   * @param message
   */
  void panic(const char* message) {
    m_has_panic = true;
  }

  //////////////////////////////////////////////////////////////

  ///////////////////
  /***
   * Odometry getters
   * @return
   */

  float distance() const {
    return m_state.total_distance;
  }

  float velocity() const {
    return m_state.velocity;
  }

  float angle() const {
    return m_state.angle;
  }

  float omega() const {
    return m_state.angular_velocity;
  }
  //////////////////////////

  void set_target_velocities(float velocity, float omega) {
    desired_velocities.velocity = velocity;
    desired_velocities.omega = omega;
  }

  /// This is safe to call only from the behaviour (mouse) code
  [[nodiscard]] VehicleState getState() const {
    return m_state;
  }

  /// This is safe to call only from the behaviour (mouse) code
  void setPose(float x, float y, float angle) {
    m_state.x = x;
    m_state.y = y;
    m_state.angle = angle;
  }

  /// This is safe to call only from the behaviour (mouse) code
  Pose getPose() {
    Pose pose;
    pose.setX(m_state.x);
    pose.setY(m_state.y);
    pose.setAngle(m_state.angle);
    return pose;
  }

  /// gets called from the application before the robot thread starts
  void setSensorCallback(SensorDataCallback callback) {
    m_sensor_callback = callback;  //
  }

  /// Once set, speeds will not change unless commanded
  /// This is safe to call only from the behaviour (mouse) code
  void setSpeeds(float velocity, float omega) {
    m_state.velocity = velocity;
    m_state.angular_velocity = omega;
  }

  /// This is safe to call only from the behaviour (mouse) code
  void setLed(const int i, const bool state) {
    const uint8_t mask = BIT(i);
    m_state.leds &= ~(mask);
    m_state.leds |= state ? mask : 0;
  }

  /// This is safe to call only from the behaviour (mouse) code
  bool readButton(Button btn) {
    return ((m_state.buttons & btn) != 0);
  }

  uint8_t getButtons() {
    return m_state.buttons;
  }

  /// this should be a single method that updates the state of the vehicle
  /// inputs like buttons, sensors, IMU, activity setting...
  /// the LEDS are included because of the simulation. They are better set
  /// directly on the target hardware.
  /// Conditional compilation can distinguish the source of this data
  void updateSensors() {
    if (m_sensor_callback) {
      m_inputs = m_sensor_callback(m_state);
      m_state.sensors = m_inputs.sensors;
      m_state.sensors.front_sum = m_state.sensors.lfs_power + m_state.sensors.rfs_power;
      m_state.sensors.front_diff = m_state.sensors.lfs_power - m_state.sensors.rfs_power;
      m_state.sensors.see_front_wall = m_inputs.sensors.front_sum > 40;
      m_state.sensors.see_left_wall = m_inputs.sensors.lfs_power > 40;
      m_state.sensors.see_right_wall = m_inputs.sensors.rfs_power > 40;
      m_state.buttons = m_inputs.buttons;
    }
  }

  /***
   * In this simulation, the updateMotion() method is invoked from the Behaviour class
   * to advance the Robot's state by one tick. Since Behaviour runs in a separate
   * thread from the main application, the Vehicle code executes in its same thread.
   * This design allows Behaviour to interact with the Robot freely, but any calls
   * from the Application to Robot or Behaviour must be thread-safe, using mutexes
   * or atomic variables.
   */
  void updateMotion(float deltaTime) {
    m_state.ticks++;
    /// The speeds are set directly from the behaviour code and now are
    /// used to update the vehicle pose.
    float deltaDistance = m_state.velocity * deltaTime;
    float deltaAngle = m_state.angular_velocity * deltaTime;

    float newX = m_state.x + deltaDistance * std::cos(m_state.angle * RADIANS);
    float newY = m_state.y + deltaDistance * std::sin(m_state.angle * RADIANS);
    float newAngle = normalizeAngle(m_state.angle + deltaAngle);

    m_state.total_distance += deltaDistance;
    m_state.x = newX;
    m_state.y = newY;
    m_state.angle = newAngle;
  }

 private:
  Vehicle(const Vehicle&) = delete;             /// no copying
  Vehicle& operator=(const Vehicle&) = delete;  /// no copying by assignment
                                                //  bool m_running;
  SensorDataCallback m_sensor_callback = nullptr;
  SystickMouseCallback systick_mouse_callback = nullptr;
  VehicleState m_state;
  VehicleInputs m_inputs;
  bool m_initialised = false;
  bool m_has_panic = false;
  float m_steering_fb = 0.0f;
  float m_step_time = 0.001f;
  Velocities desired_velocities;
};
