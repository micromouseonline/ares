//
// Created by peter on 29/01/25.
//
// Vehicle.cpp
// Implementation of the Vehicle class

#include "vehicle.h"
#include <cmath>

Vehicle::Vehicle()
    : m_state() {
  reset();
}

Vehicle::~Vehicle() {
}

void Vehicle::reset() {
  m_state.ticks = 0;
  m_state.total_distance = 0;
  setTargetVelocities(0, 0);
}

void Vehicle::begin() {
  m_state.ticks = 0;
  m_state.total_distance = 0;
  setTargetVelocities(0, 0);
  m_initialised = true;
}

void Vehicle::systick() {
  updateSensors();
  updateMotion(m_step_time);

  if (systick_mouse_callback) {
    systick_mouse_callback();
  }

  Velocities actual_velocities;
  MotorVoltages motor_voltages = motorControllersUpdate(desired_velocities, actual_velocities, m_steering_fb);
  set_motor_voltage(motor_voltages.left, motor_voltages.right);
  updateLeds();
}

void Vehicle::set_steering_feedback(float steering_fb) {
  m_steering_fb = steering_fb;
}

void Vehicle::set_systick_callback(SystickMouseCallback callback) {
  systick_mouse_callback = callback;
}

void Vehicle::updateLeds() {
  setLed(7, m_state.sensors.lfs_power > 18);
  setLed(6, m_state.sensors.lds_power > 40);
  setLed(5, m_state.sensors.rds_power > 40);
  setLed(4, m_state.sensors.rfs_power > 18);
  setLed(1, (m_state.buttons & Button::BTN_RESET) != 0);
  setLed(0, (m_state.buttons & Button::BTN_GO) != 0);
}

MotorVoltages Vehicle::motorControllersUpdate(Velocities desired, Velocities actual, float steering_feedback) {
  (void)desired;
  (void)actual;
  (void)steering_feedback;
  return {0, 0};
}

void Vehicle::set_motor_voltage(float left, float right) {
  (void)left;
  (void)right;
}

float Vehicle::battery_voltage() {
  return 7.4f;
}

bool Vehicle::has_panic() {
  return m_has_panic;
}

void Vehicle::panic(const char* message) {
  (void)message;
  m_has_panic = true;
}

float Vehicle::distance() const {
  return m_state.total_distance;
}

float Vehicle::velocity() const {
  return m_state.velocity;
}

float Vehicle::angle() const {
  return m_state.angle;
}

float Vehicle::omega() const {
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

Pose Vehicle::getPose() {
  Pose pose;
  pose.setX(m_state.x);
  pose.setY(m_state.y);
  pose.setAngle(m_state.angle);
  return pose;
}

void Vehicle::setSensorCallback(SensorDataCallback callback) {
  m_sensor_callback = callback;
}

void Vehicle::setLed(const int i, const bool state) {
  const uint8_t mask = BIT(i);
  m_state.leds &= ~(mask);
  m_state.leds |= state ? mask : 0;
}

bool Vehicle::readButton(Button btn) {
  return ((m_state.buttons & btn) != 0);
}

uint8_t Vehicle::getButtons() {
  return m_state.buttons;
}

void Vehicle::updateSensors() {
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

void Vehicle::updateMotion(float deltaTime) {
  m_state.ticks++;
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

bool Vehicle::hasButtonPressed() {
  return m_inputs.buttons != 0;
}
void Vehicle::reset_drive_system() {
  setTargetVelocities(0, 0);
  /// reset odometry
  /// disable controllers
  /// set motor voltages to zero
  /// reset motors
}
void Vehicle::setLedPattern(uint8_t pattern) {
  m_state.leds = pattern;
}
