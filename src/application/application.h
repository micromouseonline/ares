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
  Application() : m_window(conf::AppName, conf::WindowSize), m_elapsed(sf::Time::Zero), mStatisticsUpdateTime(sf::Time::Zero), m_robot() {
    RestartClock();
    m_elapsed = sf::Time::Zero;
    mStatisticsUpdateTime = sf::Time::Zero;

    /// The UI components are defined in world pixels in the window's default view
    m_textbox.Setup(5, 14, 600, sf::Vector2f(1000, 10));
    m_textbox.Add("Hello World!");
    m_window.AddObserver(this);

    /// The Lambda expression here serves to bind the callback to the application instance
    m_robot.SetSensorCallback([this]() -> SensorData { return this->CallbackCalculateSensorData(); });
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
    // Update sensor data for the robot
    m_mazeManager.UpdateObstacles();
    // now read back what the robot sees
    SensorData sensors = m_robot.GetSensorData();
    char str[100];
    sprintf(str, "%4d %4d %4d %4d ", (int)sensors.lfs_value, (int)sensors.lds_value, (int)sensors.rds_value, (int)sensors.rfs_value);
    std::string msg = std::to_string(sensors.lfs_value) + " " + std::to_string(sensors.rfs_value);
    m_textbox.Add(str);
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
    // Render the physical maze
    // TODO: think about how to add and distinguish the robot map
    //       from the physical maze
    m_mazeManager.Render(window);

    // Draw the robot using the RobotDisplay class
    sf::Vector2f pose = m_robot.GetPose();
    float orientation = m_robot.GetOrientation();
    RobotDisplay::Draw(window, pose, orientation);
    //    m_RobotBody.draw(window);
    //    m_mazeManager.UpdateObstacles();

    //    for (auto& sensor : m_robot_sensors) {
    //      sensor.draw(window);
    //    }

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
  SensorData CallbackCalculateSensorData() {
    std::vector<sf::RectangleShape> obstacles;  //= m_mazeManager.GetObstacles(); TODO: GetObstacles is broken
    sf::RectangleShape block({180.0f, 180.0f});
    block.setPosition(540, 1080);
    obstacles.push_back(block);

    static uint32_t ticks = 0;
    /// just modify the sensor values a bit to see that they change
    float v = 50.0f + (++ticks % 1000) / 10.0f;
    {
      //      std::lock_guard<std::mutex> lock(m_sensorDataMutex);
      m_SensorData.lfs_value = v;
      m_SensorData.rfs_value = v;
      m_SensorData.lds_value = v;
      m_SensorData.rds_value = v;
      m_SensorData.lfs_dist = sqrtf(v);
      m_SensorData.rfs_dist = sqrtf(v);
      m_SensorData.lds_dist = sqrtf(v);
      m_SensorData.rds_dist = sqrtf(v);
    }
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
  sf::Time m_elapsed;

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
