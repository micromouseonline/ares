//
// Created by peter on 22/11/24.
//

#ifndef ROBOT_H
#define ROBOT_H

#include <SFML/Graphics.hpp>
#include <mutex>
#include <cmath>
#include <thread>
#include "behaviour/robot-control.h"
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

/// forward declarations
class Application;

/***
 * This is the robot class.
 */
class Robot
{
public:
 Robot();
 ~Robot();

 // Starts the robot in its own thread
 void Start(Application* app);

 // Stops the robot's thread
 void Stop();

 // Accessors for robot state (thread-safe)
 sf::Vector2f GetPose() const; // Returns (x, y) position
 float GetOrientation() const; // Returns orientation in radians

 // This must request sensor data from the Application
 void ReadSensorValues();

 /// A getter for the Robot's sensor values
 SensorValues& GetSensorValues();

 void StartSystick();

 void StopSystick();

 void systick();

 uint32_t millis();

private:
 // Main loop executed in the robot thread
 void Run();

 // Low-level control methods
 void UpdateState();
 void ProcessControl();
 float lfs = 0.0f;
 float lds = 0.0f;
 float rds = 0.0f;
 float rfs = 0.0f;
 uint32_t ticks = 0;
 std::thread m_systick_thread;
 std::thread m_thread;
 std::atomic<bool> m_running; // Thread control flag
 float m_AngularVelocity = 0.0f;
 float m_LinearVelocity = 0.0f;
 float m_LoopTime = 0.001f;
 sf::Vector2f m_pose; // (x, y) position
 float m_orientation; // Orientation in radians

 mutable std::mutex m_SystickMutex; // Protects access to m_pose and m_orientation
 SensorValues m_sensorValues; // Current sensor readings
 RobotControl m_control; // Higher-level control logic
 Application* m_application = nullptr; // Pointer to the Application (for querying sensor readings)
};

#endif  // ROBOT_H
