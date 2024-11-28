//
// Created by peter on 23/11/24.
//

#include "robot.h"
#include <iostream>
#include <mutex>
#include <thread>
#include "application/application.h" // WARN: this is a bad dependency

Robot::Robot()
  : m_running(false),
    m_pose(1300.0f, 1300.0f),
    m_orientation(0.0f) {}

Robot::~Robot() { Stop(); }

/***
 * The main application should call the Start() method to start the robot, passing
 * in a pointer to itself so that callbacks can be used. Fo example, the robot
 * can request sensor data from the application.
 *
 * Once started, the robot will run until Stop() is called.
 *
 * This robot instance will also start its own thread that repeatedly calls the
 * systick() method. In this case, systick runs at about 1kHz. Actually a little
 * less because we run the code then sleep for 1ms. Although the timing will not
 * be completely accurate, it should suffice for simulation.
 *
 * We could
 *
 * @param app - the calling thread.
 */
void Robot::Start(Application* app) {
  if (!m_running) {
    m_application = app;
    m_running = true;
    m_thread = std::thread(&Robot::Run, this);
    m_systick_thread = std::thread(&Robot::systick, this);
  }
}

void Robot::Stop() {
  if (m_running) {
    m_running = false;
    if (m_systick_thread.joinable()) { m_systick_thread.join(); }
    if (m_thread.joinable()) { m_thread.join(); }
  }
}

/***
 * The systick() method runs in its own thread, with the loop running at, in
 * this case, about 1kHz. The systick() method simulates an interrupt triggered
 * by a timer on the hardware.
 *
 * In the real hardware, operations during the Interrupt Service Routine (ISR) are
 * mutually exclusive with the main code. This would not normally be the case here,
 * so everything that happens in systick() is guarded with a mutex. This ensures
 * the code in systick() can run and modify state as needed, while the top-level
 * robot code will block as soon as it tries to access any of the resources guarded
 * by the mutex, thus simulating the behavior of the hardware interrupt.
 *
 * If the top-level code needs to guarantee that systick() will not change any
 * shared state, it can also lock the same mutex for the duration of a critical
 * section. If the mutex is already locked, the top-level code will block until
 * systick() has finished that iteration. Then the top-level code will acquire the
 * mutex and block systick() until it is released.
 *
 * Care must be taken to ensure that these mutually cooperative functions do
 * not end up in a deadlock.
 */

void Robot::systick() {
  using namespace std::chrono;
  /// NOTE: this runs a little fast on linux
  auto interval_us = duration_cast<microseconds>(duration<float>(m_LoopTime));
  auto next_time = steady_clock::now() + interval_us;
  while (m_running) {
    // The mutex will lock out the main thread while this block runs.
    CRITICAL_SECTION(m_SystickMutex) {
      ticks++;
      // MonitorEvents();
      ReadSensorValues();
      // UpdateMotion();
      m_orientation += m_AngularVelocity * m_LoopTime;
      if (m_orientation >= 360.0f) { m_orientation -= 360.0f; } else
        if (m_orientation < 0.0f) { m_orientation += 360.0f; }
      // Update position based on linear velocity and orientation
      float orientationRad = m_orientation * RADIANS;
      m_pose.x += m_LinearVelocity * std::cos(orientationRad) * m_LoopTime;
      m_pose.y += m_LinearVelocity * std::sin(orientationRad) * m_LoopTime;
    }
    std::this_thread::sleep_until(next_time);
    next_time += interval_us;
  }
}

/***
 * The run() method is what gets called from the thread initialisation
 * It should run continuously when started. An atomic flag, m_running,
 * is used so that the parent thread can signal an orderly termination.
 */
void Robot::Run() {
  /// TODO: for testing, just have the robot run around in a circle.
  ///       This will be replaced with the real robot code
  const sf::Vector2f initialCenter(2200.0f, 1200.0f); // Center of the circle
  const float initialRadius = 400.0f;                 // Radius of the circle
  const float initialAngularSpeed = 120.0f;           // Degrees per second

  m_control.SetCircularTrajectory(initialCenter, initialRadius, initialAngularSpeed);
  sf::Clock clock;
  while (m_running) {
    // Get the velocities from the RobotControl class
    m_LinearVelocity = m_control.GetLinearVelocity();
    m_AngularVelocity = m_control.GetAngularVelocity();

    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100 Hz loop
  }
}

void Robot::UpdateState() {
  std::lock_guard<std::mutex> lock(m_SystickMutex);
  // Update the robot's position and orientation (simulation logic here)
}

void Robot::ProcessControl() {
  // Call RobotControl to compute control actions based on the sensor data
  m_control.Update(m_sensorValues);
}

sf::Vector2f Robot::GetPose() const {
  std::lock_guard<std::mutex> lock(m_SystickMutex);
  return m_pose;
}

float Robot::GetOrientation() const {
  std::lock_guard<std::mutex> lock(m_SystickMutex);
  return m_orientation;
}

void Robot::ReadSensorValues() { if (m_application) { m_sensorValues = m_application->GetSensorValues(); } }

SensorValues& Robot::GetSensorValues() {
  std::lock_guard<std::mutex> lock(m_SystickMutex);
  return m_sensorValues;
}

uint32_t Robot::millis() {
  std::lock_guard<std::mutex> lock(m_SystickMutex);
  return ticks;;
}
