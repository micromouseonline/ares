//
// Created by peter on 22/11/24.
//

#ifndef ROBOT_H
#define ROBOT_H

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include "behaviour/behaviour.h"
#include "common/core.h"
#include "sensor-data.h"

/***
 * TODO Not sure this is the best place for this macro
 *
 * the macro looks odd because it uses a C++17 feature where
 * the if statement condition can include an initialiser.
 * This feature allows you to initialize variables within the
 * if statement, followed by a condition that determines if
 * the block should be executed. It's a neat way to declare
 * and initialise variables that are only used within the
 * scope of the if statement. Use it like this:
 *
 * if (initialiser; condition){
 *   // code that will run if condition is true
 * }
 *
 * The scope of any variables declared in the initializer
 * of an if statement is limited to the entire if block,
 * including both the condition and the code within the
 * braces {} of the if statement.
 *
 * Use the macro like this
 *
 * CRITICAL_SECTION(m_systick_mutex){
 *   // guarded code
 * }
 *
 */

#define CRITICAL_SECTION(the_mutex) if (std::lock_guard<std::mutex> lock(the_mutex); true)

/***
 * This is the robot class.
 */
class Robot {
 public:
  Robot() : m_running(false), m_pose(1300.0f, 1300.0f), m_orientation(0.0f) {
    for (int i = 0; i < conf::SENSOR_COUNT; i++) {
      m_SensorOffsets[i] = conf::SensorDefaultOffsets[i];  //
    }
  }

  ~Robot() { Stop(); }

  // Starts the robot in its own thread

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
  void Start() {
    if (!m_running) {
      m_running = true;
      m_thread = std::thread(&Robot::Run, this);
      m_systick_thread = std::thread(&Robot::systick, this);
    }
  }

  // Stops the robot's thread
  void Stop() {
    if (m_running) {
      m_running = false;
      if (m_systick_thread.joinable()) {
        m_systick_thread.join();
      }
      if (m_thread.joinable()) {
        m_thread.join();
      }
    }
  }

  // Accessors for robot state (thread-safe)
  sf::Vector2f GetPose() const {
    std::lock_guard<std::mutex> lock(m_SystickMutex);
    return m_pose;
  }

  float GetOrientation() const {
    std::lock_guard<std::mutex> lock(m_SystickMutex);
    return m_orientation;
  }

  void SetSensorCallback(SensorDataCallback callback) { m_SensorCallback = callback; }

  // This must request sensor data from the Application
  void ReadSensorData();

  /// A getter for the Robot's sensor values

  SensorData& GetSensorData() {
    std::lock_guard<std::mutex> lock(m_SystickMutex);
    return m_SensorData;
  }

  void StartSystick();

  void StopSystick();

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

  void systick() {
    using namespace std::chrono;
    /// NOTE: this runs a little fast on linux
    auto interval_us = duration_cast<microseconds>(duration<float>(m_LoopTime));
    auto next_time = steady_clock::now() + interval_us;
    while (m_running) {
      // The mutex will lock out the main thread while this block runs.
      CRITICAL_SECTION(m_SystickMutex) {
        ticks++;
        // MonitorEvents();
        if (m_SensorCallback) {
          m_SensorData = m_SensorCallback();
        }
        // UpdateMotion();
        m_orientation += m_AngularVelocity * m_LoopTime;
        if (m_orientation >= 360.0f) {
          m_orientation -= 360.0f;
        } else if (m_orientation < 0.0f) {
          m_orientation += 360.0f;
        }
        // Update position based on linear velocity and orientation
        float orientationRad = m_orientation * RADIANS;
        m_pose.x += m_LinearVelocity * std::cos(orientationRad) * m_LoopTime;
        m_pose.y += m_LinearVelocity * std::sin(orientationRad) * m_LoopTime;
      }
      std::this_thread::sleep_until(next_time);
      next_time += interval_us;
    }
  }

  uint32_t millis() {
    std::lock_guard<std::mutex> lock(m_SystickMutex);
    return ticks;
    ;
  }

 private:
  /***
   * The run() method is what gets called from the thread initialisation
   * It should run continuously when started. An atomic flag, m_running,
   * is used so that the parent thread can signal an orderly termination.
   */
  void Run() {
    /// TODO: for testing, just have the robot run around in a circle.
    ///       This will be replaced with the real robot code
    const sf::Vector2f initialCenter(2200.0f, 1200.0f);  // Center of the circle
    const float initialRadius = 400.0f;                  // Radius of the circle
    const float initialAngularSpeed = 120.0f;            // Degrees per second

    m_control.SetCircularTrajectory(initialCenter, initialRadius, initialAngularSpeed);
    sf::Clock clock;
    while (m_running) {
      // Get the velocities from the RobotControl class
      m_LinearVelocity = m_control.GetLinearVelocity();
      m_AngularVelocity = m_control.GetAngularVelocity();

      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100 Hz loop
    }
  }

  //  // Low-level control methods
  //  void UpdateState() {
  //    std::lock_guard<std::mutex> lock(m_SystickMutex);
  //    // Update the robot's position and orientation (simulation logic here)
  //  }
  //  void ProcessControl();

  uint32_t ticks = 0;
  std::thread m_systick_thread;
  std::thread m_thread;
  std::atomic<bool> m_running;  // Thread control flag
  float m_AngularVelocity = 0.0f;
  float m_LinearVelocity = 0.0f;
  float m_LoopTime = 0.001f;
  sf::Vector2f m_pose;  // (x, y) position
  float m_orientation;  // Orientation in radians

  SensorDataCallback m_SensorCallback = nullptr;
  mutable std::mutex m_SystickMutex;  // Protects access to m_pose and m_orientation
  SensorData m_SensorData;            // Current sensor readings

  Behaviour m_control;  // Higher-level control logic
  SensorGeometry m_SensorOffsets[conf::SENSOR_COUNT];
};

#endif  // ROBOT_H
