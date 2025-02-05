//
// Created by peter on 29/01/25.
//
// Vehicle.cpp
// Implementation of the Vehicle class

#include "vehicle.h"
#include <cmath>
#include "behaviour/config.h"
#include "board/hal/motor-controller.h"
#include "robot/board/board-mr32.h"
//
// Vehicle::Vehicle()
//    : m_state() {
//  if (!m_initialised) {
//    init();
//  }
//  resetDriveSystem();
//}

Vehicle::~Vehicle() {
}

void Vehicle::init() {
  m_state.ticks = 0;
  m_state.total_distance = 0;
  resetDriveSystem();
  m_initialised = true;
  m_reset = false;
}

bool Vehicle::systick() {
  if (m_terminate || m_reset || m_paused) {
    return false;
  }
  m_state.ticks++;
  updateSensors();

  if (m_SystickMouseCallback) {
    m_SystickMouseCallback();
  }

  updateVehiclePose(m_step_time);
  Velocities actual_velocities;
  MotorVoltages motor_voltages = updateMotorControlllers(desired_velocities, actual_velocities, m_steering_fb);
  setMotorVoltage(motor_voltages.left, motor_voltages.right);
  return true;
}

void Vehicle::setSteeringFeedback(float steering_fb) {
  m_steering_fb = steering_fb;
}

void Vehicle::setMouseCallback(SystickMouseCallback callback) {
  m_SystickMouseCallback = callback;
}

MotorVoltages Vehicle::updateMotorControlllers(Velocities desired, Velocities actual, float steering_feedback) {
  (void)desired;
  (void)actual;
  (void)steering_feedback;
  return {0, 0};
}

void Vehicle::setMotorVoltage(float left, float right) {
  (void)left;
  (void)right;
}

float Vehicle::getBatteryVoltage() {
  return 7.4f;
}

bool Vehicle::panicIsActive() {
  return m_has_panic;
}

void Vehicle::panic(const char* message) {
  (void)message;
  m_has_panic = true;
}

float Vehicle::getDistance() const {
  return m_state.total_distance;
}

float Vehicle::getVelocity() const {
  return m_state.velocity;
}

float Vehicle::getAngle() const {
  return m_state.angle;
}

float Vehicle::getOmega() const {
  return m_state.angular_velocity;
}

void Vehicle::setTargetVelocities(float velocity, float omega) {
  desired_velocities.velocity = velocity;
  desired_velocities.omega = omega;
  m_state.velocity = velocity;
  m_state.angular_velocity = omega;
}

VehicleState Vehicle::getState() const {
  return m_state;
}

void Vehicle::setPose(float x, float y, float angle) {
  m_state.x = x;
  m_state.y = y;
  m_state.angle = angle;
}

void Vehicle::setSensorCallback(SensorDataCallback callback) {
  m_SensorReadCallback = callback;
}

void Vehicle::setLed(const int i, const bool state) {
  const uint8_t mask = BIT(i);
  m_state.leds &= ~(mask);
  m_state.leds |= state ? mask : 0;
}

// bool Vehicle::readButton(Button btn) {
//   return ((m_state.buttons & btn) != 0);
// }

void Vehicle::updateSensors() {
  /// TODO move this to the mouse
  if (m_SensorReadCallback) {
    m_inputs = m_SensorReadCallback(m_state);

    m_state.sensors.lfs_power = m_inputs.adc[LFS_ADC_CHANNEL];
    m_state.sensors.lds_power = m_inputs.adc[LDS_ADC_CHANNEL];
    m_state.sensors.rds_power = m_inputs.adc[RDS_ADC_CHANNEL];
    m_state.sensors.rfs_power = m_inputs.adc[RFS_ADC_CHANNEL];
    m_state.sensors.front_sum = m_state.sensors.lfs_power + m_state.sensors.rfs_power;
    m_state.sensors.front_diff = m_state.sensors.lfs_power - m_state.sensors.rfs_power;
    m_state.sensors.see_front_wall = m_inputs.sensors.front_sum > 40;
    m_state.sensors.see_left_wall = m_inputs.sensors.lfs_power > 40;
    m_state.sensors.see_right_wall = m_inputs.sensors.rfs_power > 40;
    m_state.buttons = m_inputs.buttons;
  }
}

void Vehicle::updateVehiclePose(float deltaTime) {
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

bool Vehicle::hasAnyButtonPressed() {
  return m_inputs.buttons != 0;
}

bool Vehicle::isButtonPressed(int button) {
  return ((m_state.buttons & button) != 0);
}

void Vehicle::resetDriveSystem() {
  setTargetVelocities(0, 0);
  /// reset odometry angle to zero and wheel counters to zero
  /// set motor voltages to zero
  /// disable controllers
  /// reset motor controller errors to zero
}

void Vehicle::setLedPattern(uint8_t pattern) {
  m_state.leds = pattern;
}

bool Vehicle::isRunning() {
  return !m_terminate && !m_reset;
}

void Vehicle::terminate() {
  m_terminate = true;
}

void Vehicle::reset() {
  m_reset = true;
}

void Vehicle::pause() {
  m_paused = true;
}
void Vehicle::resume() {
  m_paused = false;
}
bool Vehicle::isPaused() {
  return m_paused;
}

///////////////////////////////
/// passthroughs for Board capabilities

Speaker* Vehicle::speaker() {
  return &m_speaker;
}

Display* Vehicle::display() {
  return &m_display;
}

Usart* Vehicle::serial() {
  return &m_serial;
}

Button* Vehicle::button_x() {
  return &m_button_x;
}

Button* Vehicle::button_y() {
  return &m_button_y;
}

AnalogueConverter* Vehicle::adc() {
  return &m_adc;
}

Battery* Vehicle::battery() {
  return &m_battery;
}
Gyro* Vehicle::gyro() {
  return &m_gyro;
}

Odometry* Vehicle::odometry() {
  return &m_odometry;
}

MotorController* Vehicle::motors() {
  return &m_motors;
}

MotorPWM* Vehicle::pwm() {
  return &m_pwm;
}
