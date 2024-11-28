//
// Created by peter on 22/11/24.
//

#ifndef APPLICATION_H
#define APPLICATION_H

#include <SFML/Graphics.hpp>
#include <cstdlib>
#include "application/event_observer.h"
#include "application/maze-manager.h"
#include "application/textbox.h"
#include "application/window.h"
#include "robot/robot.h"
#include "robot/sensor-data.h"

class Application : public IEventObserver
{
public:
 Application();
 ;

 ~Application();

 void Run();
 ;

 void OnEvent(const Event& event) override;

 /***
  * Note that this is not about the events. We can test the state of the mouse and
  * the keyboard or any other input devices.
  * We are not looking for things that have happened like keypress events.
  * Could this get tricky if we want to do something like enter text? We will see.
  */
 void HandleInput();

 /// This is where the work gets done. All the application logic and behaviour
 /// is performed from this method. By passing in a deltaTime we can choose to
 /// perform updates once per frame or at a higher, or lower, fixed time interval.
 /// No rendering is done from this method.
 void Update(sf::Time deltaTime = sf::seconds(0.01));

 /// The Render() method is the only place that output is generated for the
 /// window - and any audio devices if used. It is called after the Update()
 /// method and will may be called once per frame or after every update depending
 /// on the underlying update strategy.
 void Render();

 Window* GetWindow();

 sf::Time GetElapsed();

 void RestartClock();

 /// This would be a good place to create any overlay information or to log
 /// performance data for example.
 void UpdateStatistics(sf::Time elapsedTime);

 void CalculateSensorValues();

 /// the robot calls this to get new sensor values
 SensorValues& GetSensorValues();

private:
 sf::Vector2f mp = {0, 0};
 static const sf::Time TimePerFrame;
 Window m_window;
 sf::Clock m_clock;
 sf::Clock m_timer;
 sf::Time m_elapsed;

 //  sf::Font mFont;
 sf::Text mStatisticsText;
 sf::Time mStatisticsUpdateTime;
 Textbox m_textbox;
 Robot m_robot; // The robot instance
 std::mutex m_sensorDataMutex; // Protects sensor data updates
 SensorValues m_sensorValues; // Current sensor reading that are read by robot
 MazeManager m_mazeManager; // This is the maze manager
};

#endif  // APPLICATION_H
