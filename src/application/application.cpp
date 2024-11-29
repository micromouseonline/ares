//
// Created by peter on 23/11/24.
//
#include "application.h"
#include <sstream>
#include "configuration.h"
#include "robot-display.h"
#include "vec2.h"
Application::Application() : m_window(conf::AppName, conf::WindowSize), m_elapsed(sf::Time::Zero), mStatisticsUpdateTime(sf::Time::Zero), m_robot() {
  RestartClock();
  m_elapsed = sf::Time::Zero;
  mStatisticsUpdateTime = sf::Time::Zero;

  /// The UI components are defined in world pixels in the window's default view
  m_textbox.Setup(5, 14, 600, sf::Vector2f(1000, 10));
  m_textbox.Add("Hello World!");
  m_window.AddObserver(this);
  m_robot.Start(this);
}

Application::~Application() {
  m_robot.Stop();  // Ensure the robot thread stops
}

/***
 * This method will be called every frame. No rendering is done.
 * It should handle user IO and any other application logic
 * You could grab the robot state for later use. Logged
 * messages can be hoovered up for permanent storage.
 *
 * Here, for example, the robot sensor values are gathered so
 * that they can be displayed in a text box
 * @param deltaTime
 */
void Application::Update(sf::Time deltaTime) {
  m_window.Update();  // call this first to process window events
  m_elapsed += deltaTime;
  // Update sensor data for the robot
  CalculateSensorValues();
  // now read back what the robot sees
  SensorValues sensors = m_robot.GetSensorValues();
  char str[100];
  sprintf(str, "%4d %4d %4d %4d ", (int)sensors.lfs_value, (int)sensors.lds_value, (int)sensors.rds_value, (int)sensors.rfs_value);
  std::string msg = std::to_string(sensors.lfs_value) + " " + std::to_string(sensors.rfs_value);
  //  m_textbox.Add(str);
}

void Application::Render() {
  /// ALWAYS do this first
  m_window.BeginDraw();
  // grab the window reference to save typing
  sf::RenderWindow& window = *m_window.GetRenderWindow();
  window.setView(m_window.getMazeView());
  // Render the physical maze
  // TODO: think about how to add and distinguish the robot map
  //       from the physical maze
  m_mazeManager.Render(window);

  // Draw the robot using the RobotDisplay class
  sf::Vector2f pose = m_robot.GetPose();
  float orientation = m_robot.GetOrientation();
  RobotDisplay::Draw(*m_window.GetRenderWindow(), pose, orientation);

  window.setView(window.getDefaultView());
  // we can draw anything else we want here.
  // it could be a path to the goal, overlaid telemetry
  // flooding values, highlight to current target
  // use your imagination.

  // the textbox it just a demonstration.
  // we can also put ImGui components here I guess
  m_textbox.Render(window);
  /// ALWAYS do this last
  m_window.EndDraw();
}

/***
 * HandleInput is used to test the STATE of an input device. It is
 * not meant to be used to respond to input events.
 *
 * For example, you might use something like this to continuously
 * move a character in a game
 *    if (sf::Keyboard::IsKeyPressed(sf::Keyboard::W) {
 *        // move forward
 *    }
 */
void Application::HandleInput() {
  /// This is a silly example of how HandleInput can be used
  if (sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
    m_textbox.SetBackgroundColor(sf::Color(255, 255, 0, 48));
  } else {
    m_textbox.SetBackgroundColor(sf::Color(255, 255, 255, 48));
  }
}

/***
 * OnEvent is used to respond to CHANGES in state such as keypress
 *
 * For example, you might use something like this to continuously
 * move a character in a game
 *    if (sf::Keyboard::IsKeyPressed(sf::Keyboard::W) {
 *        // move forward
 *    }
 */

sf::FloatRect getViewBounds(const sf::RenderWindow& window, const sf::View& view) {
  sf::Vector2f viewSize = view.getSize();
  sf::Vector2f viewCenter = view.getCenter();
  sf::Vector2f viewCorner = viewCenter - (viewSize / 2.0f);
  return {viewCorner, viewSize};
}

void Application::OnEvent(const Event& event) {
  switch (event.type) {
    case EventType::SFML_EVENT:
      if (event.event.type == sf::Event::MouseButtonPressed) {
        sf::RenderWindow* window = m_window.GetRenderWindow();
        sf::Vector2i pixelPos(sf::Mouse::getPosition(*window));
        sf::Vector2f mazePos{0, 0};
        window->setView(m_window.getMazeView());
        sf::View mazeView = m_window.getMazeView();  // TODO: where should we store the mazeView? In config?
        sf::FloatRect mazeBounds = getViewBounds(*window, conf::MazeView);
        if (mazeBounds.contains(float(pixelPos.x), float(pixelPos.y))) {
          mazePos = (window->mapPixelToCoords(pixelPos, mazeView));
        }

        std::stringstream msg;
        if (event.event.mouseButton.button == sf::Mouse::Right) {
          msg << " (RIGHT BUTTON) ";
        } else {
          msg << " (OTHER BUTTON) ";
        }
        msg << " @ " << Vec2(mazePos) << " =  " << Vec2(pixelPos);
        m_textbox.Add(msg.str());
      }
      break;
    case EventType::USER_EVENT:
      m_textbox.Add("USER Event");
      break;
    default:
      m_textbox.Add("UNHANDLED Event");
      break;
  }
}

void Application::Run() {
  while (!GetWindow()->IsDone()) {
    HandleInput();
    Update();
    Render();
    RestartClock();
  }
}

void Application::UpdateStatistics(sf::Time elapsedTime) {
  (void)(elapsedTime);
  //
}

Window* Application::GetWindow() {
  return &m_window;  //
}

sf::Time Application::GetElapsed() {
  return m_elapsed;  //
}

void Application::RestartClock() {
  m_clock.restart();  //
}

void Application::CalculateSensorValues() {
  uint32_t ticks = m_robot.millis();
  /// just modify the sensor values a bit to see that they change
  float v = 50.0f + (ticks % 1000) / 10.0f;
  {
    std::lock_guard<std::mutex> lock(m_sensorDataMutex);
    m_sensorValues.lfs_value = v;
    m_sensorValues.rfs_value = v;
    m_sensorValues.lds_value = v;
    m_sensorValues.rds_value = v;
    m_sensorValues.lfs_dist = sqrtf(v);
    m_sensorValues.rfs_dist = sqrtf(v);
    m_sensorValues.lds_dist = sqrtf(v);
    m_sensorValues.rds_dist = sqrtf(v);
  }
}

SensorValues& Application::GetSensorValues() {
  return m_sensorValues;
}
