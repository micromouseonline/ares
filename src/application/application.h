//
// Created by peter on 22/11/24.
//

#ifndef APPLICATION_H
#define APPLICATION_H

#include <SFML/Graphics.hpp>
#include <cstdlib>
#include "application/event_observer.h"
#include "application/maze-manager.h"
#include "application/robot-wall-sensor.h"
#include "application/textbox.h"
#include "application/window.h"
#include "common/core.h"
#include "common/vec2.h"
#include "robot-body.h"
#include "robot/robot.h"
#include "robot/sensor-data.h"

class Application : public IEventObserver {
 public:
  Application() : m_window(conf::AppName, conf::WindowSize), m_elapsed(sf::Time::Zero), mStatisticsUpdateTime(sf::Time::Zero) {
    RestartClock();
    m_elapsed = sf::Time::Zero;
    mStatisticsUpdateTime = sf::Time::Zero;

    m_appFont.loadFromFile("assets/fonts/ubuntu-mono-r.ttf");

    m_adhocText.setFont(m_appFont);
    m_adhocText.setCharacterSize(16);
    m_adhocText.setFillColor(sf::Color::Yellow);

    /// The UI components are defined in world pixels in the window's default view
    m_textbox.Setup(5, 14, 600, sf::Vector2f(1000, 10));
    m_textbox.Add("Hello World!");
    m_textbox.Add("WASD keys to move robot");

    m_window.AddObserver(this);
    m_RobotBody.setRobot(m_robot);

    m_robot.setPosition(7 * CELL_SIZE + (CELL_SIZE + WALL_THICKNESS) / 2.0f, 8 * CELL_SIZE + (CELL_SIZE + WALL_THICKNESS) / 2.0f);
    m_robot.setOrientation(0.0);
    /// The Lambda expression here serves to bind the callback to the application instance
    m_robot.setSensorCallback([this](float x, float y, float theta) -> SensorData { return this->CallbackCalculateSensorData(x, y, theta); });
    m_robot.Start();
  }

  ~Application() {
    m_robot.Stop();  // Ensure the robot thread stops
  }

  void Run() {
    while (!GetWindow()->IsDone()) {
      HandleInput();
      Update();
      Render();
      RestartClock();
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

  void OnEvent(const Event& event) override {
    switch (event.type) {
      case EventType::SFML_EVENT:
        if (event.event.type == sf::Event::MouseButtonPressed) {
          sf::RenderWindow* window = m_window.GetRenderWindow();
          sf::Vector2i pixelPos(sf::Mouse::getPosition(*window));
          sf::Vector2f mazePos{0, 0};
          window->setView(m_window.getMazeView());
          sf::View mazeView = m_window.getMazeView();  // TODO: where should we store the mazeView? In config?
          sf::FloatRect mazeBounds = getViewBounds(conf::MazeView);
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

  /***
   * Note that this is not about the events. We can test the state of the mouse and
   * the keyboard or any other input devices.
   * We are not looking for things that have happened like keypress events.
   * Could this get tricky if we want to do something like enter text? We will see.
   *
   * HandleInput is used to test the STATE of an input device. It is
   * not meant to be used to respond to input events.
   *
   * For example, you might use something like this to continuously
   * move a character in a game
   *    if (sf::Keyboard::IsKeyPressed(sf::Keyboard::W) {
   *        // move forward
   *    }
   */
  void HandleInput() {
    /// This is a silly example of how HandleInput can be used
    if (sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
      m_textbox.SetBackgroundColor(sf::Color(255, 255, 0, 48));
    } else {
      m_textbox.SetBackgroundColor(sf::Color(255, 255, 255, 48));
    }
    float v = 0;
    float w = 0;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
      w = 90;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
      w = -90;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
      v = 540;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
      v = -540;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift)) {
      w = w / 10.0f;
      v = v / 10.0f;
    }
    m_robot.setVelocity(v);
    m_robot.setOmega(w);
  }

  /***
   * This is where the work gets done. All the application logic and behaviour
   * is performed from this method. By passing in a deltaTime we can choose to
   * perform updates once per frame or at a higher, or lower, fixed time interval.
   * No rendering is done from this method.
   * This method will be called every frame. No rendering is done.
   * It should handle user IO and any other application logic
   * You could grab the robot state for later use. Logged
   * messages can be hoovered up for permanent storage.
   *
   * Here, for example, the robot sensor values are gathered so
   * that they can be displayed in a text box
   * @param deltaTime
   */
  void Update(sf::Time deltaTime = sf::seconds(0.01)) {
    m_window.Update();  // call this first to process window events
    m_elapsed += deltaTime;
    std::string msg;
    SensorData sensors = m_robot.getSensorData();
    char str[100];
    sprintf(str, "%4d %4d %4d %4d ", (int)sensors.lfs_value, (int)sensors.lds_value, (int)sensors.rds_value, (int)sensors.rfs_value);
    msg += str;
    msg += "\n";
    msg += "sensor update: " + std::to_string(m_processTime.asMicroseconds()) + " us";
    msg += "\n";
    float angle = m_robot.getOrientation();
    sf::Vector2f pos = m_robot.getPose();
    sprintf(str, "Robot: (%4d,%4d) %4d\n", (int)pos.x, (int)pos.y, (int)angle);
    msg += str;
    m_adhocText.setString(msg);
  }

  /// The Render() method is the only place that output is generated for the
  /// window - and any audio devices if used. It is called after the Update()
  /// method and will may be called once per frame or after every update depending
  /// on the underlying update strategy.

  void Render() {
    /// ALWAYS do this first
    m_window.BeginDraw();
    // grab the window reference to save typing
    sf::RenderWindow& window = *m_window.GetRenderWindow();
    window.setView(m_window.getMazeView());
    // Render the physical maze TODO: think about how to add and distinguish the robot map from the physical maze
    m_mazeManager.Render(window);
    m_RobotBody.draw(window);

    window.setView(window.getDefaultView());
    // we can draw anything else we want here.

    m_adhocText.setPosition(1000, 200);
    window.draw(m_adhocText);

    m_textbox.Render(window);
    /// ALWAYS do this last
    m_window.EndDraw();
  }

  Window* GetWindow() { return &m_window; }

  sf::Time GetElapsed() { return m_elapsed; }

  void RestartClock() {
    m_clock.restart();  //
  }

  /// This would be a good place to create any overlay information or to log
  /// performance data for example.
  void UpdateStatistics(sf::Time elapsedTime) {
    (void)elapsedTime;  //
  }

  /**
   * This is the callback that calculates the sensor data for the robot.
   * It will use the Robot pose and the Sensor geometry to work out what
   * the sensors would see in the world maze.
   *
   * We return a COPY of the data because the caller, the Robot, runs in
   * a different thread and it is easier, although a little slower, to
   * ensure thread safety this way.
   *
   * It may be best to return only the RAW values because that is what
   * the Robot actually works with. Effectively simulate the ADC with
   * ambient light cancellation.
   *
   * NOTE: is there any need to retain the locally calculated values?
   *       We should probably only ask the Robot what is has.
   *
   * NOTE: Do not call any robot functions in here or there may be
   *       a deadlock
   *
   * @return a copy of the local sensor data
   */
  SensorData CallbackCalculateSensorData(float x, float y, float theta) {
    m_timer.restart();
    m_RobotBody.updateSensorGeometry(x, y, theta);
    m_RobotBody.updateSensors(m_mazeManager.GetObstacles());
    m_SensorData.lfs_value = m_RobotBody.getSensor(conf::LFS).power();
    m_SensorData.rds_value = m_RobotBody.getSensor(conf::LDS).power();
    m_SensorData.lds_value = m_RobotBody.getSensor(conf::RDS).power();
    m_SensorData.rfs_value = m_RobotBody.getSensor(conf::RFS).power();
    m_processTime = m_timer.getElapsedTime();
    return m_SensorData;
  }

 private:
  sf::FloatRect getViewBounds(const sf::View& view) {
    sf::Vector2f viewSize = view.getSize();
    sf::Vector2f viewCenter = view.getCenter();
    sf::Vector2f viewCorner = viewCenter - (viewSize / 2.0f);
    return {viewCorner, viewSize};
  }

  sf::Vector2f mp = {0, 0};
  static const sf::Time TimePerFrame;
  Window m_window;
  sf::Clock m_clock;
  sf::Clock m_timer;
  sf::Time m_processTime;  // used for timing code fragments
  sf::Time m_elapsed;

  sf::Font m_appFont;
  sf::Text m_adhocText;  // use this for adhoc messages, overlays and the like

  //  sf::Font mFont;
  sf::Text mStatisticsText;
  sf::Time mStatisticsUpdateTime;
  Textbox m_textbox;
  Robot m_robot;  // The robot instance
  RobotBody m_RobotBody;

  std::mutex m_sensorDataMutex;  // Protects sensor data updates
  SensorData m_SensorData;       // Current sensor reading that are read by robot
  MazeManager m_mazeManager;     // This is the maze manager
  std::mutex appMutex;
};

#endif  // APPLICATION_H
