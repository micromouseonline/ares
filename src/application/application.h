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
#include "imgui-SFML.h"
#include "imgui.h"
#include "robot-body.h"
#include "robot/robot.h"
#include "robot/sensor-data.h"

class Application : public IEventObserver {
 public:
  Application() : m_window(std::make_unique<Window>(conf::AppName, conf::WindowSize)), m_elapsed(sf::Time::Zero) {
    m_elapsed = sf::Time::Zero;

    m_default_font.loadFromFile("assets/fonts/ubuntu-mono-r.ttf");

    m_adhoc_text.setFont(m_default_font);
    m_adhoc_text.setCharacterSize(16);
    m_adhoc_text.setFillColor(sf::Color::Yellow);
    m_txt_maze_name.setFont(m_default_font);
    m_txt_maze_name.setCharacterSize(16);
    m_txt_maze_name.setFillColor(sf::Color::Yellow);
    m_txt_maze_name.setPosition(10, 973);

    /// The UI components are defined in world pixels in the window's default view
    m_textbox.initialise(5, 14, 600, sf::Vector2f(1000, 10));
    m_textbox.addString("Hello World!");
    m_textbox.addString("WASD keys to move robot");

    //    m_maze_manager.loadFromMemory(japan2007ef, 16);
    //    m_maze_manager.loadFromMemory(japan2016ef_half, 32);

    /// Have the window inform us of any events
    m_window->addObserver(this);

    if (!ImGui::SFML::Init(*m_window->getRenderWindow())) {
      std::cerr << "ImGui failed to initialise" << std::endl;
    }

    for (int i = 0; i < mazeCount; i++) {
      m_maze_names.push_back(mazeList[i].title);
    }
    static int maze_index = 0;

    /// set up the robot
    m_robot_body.setRobot(m_robot);
    sf::Vector2f start_pos = m_maze_manager.getCellCentre(0, 0);
    m_robot.setPosition(start_pos.x, start_pos.y);
    m_robot.setOrientation(90.0);
    /// The Lambda expression here serves to bind the callback to the application instance
    m_robot.setSensorCallback([this](float x, float y, float theta) -> SensorData { return this->callbackCalculateSensorData(x, y, theta); });
    m_robot.Start();
  }

  ~Application() {
    m_robot.Stop();    // Ensure the robot thread stops
    m_window.reset();  // destroys the widow explicitly so that we can clean up
    ImGui::SFML::Shutdown();
  }

  void run() {
    while (!getWindow()->isDone()) {
      handleInput();
      update();
      render();
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

  void onEvent(const AppEvent& event) override {
    ImGui::SFML::ProcessEvent(*m_window->getRenderWindow(), event.event);
    switch (event.type) {
      case EventType::SFML_EVENT:
        if (event.event.type == sf::Event::MouseButtonPressed) {
          sf::RenderWindow* window = m_window->getRenderWindow();
          sf::Vector2i pixelPos(sf::Mouse::getPosition(*window));
          sf::Vector2f mazePos{0, 0};
          window->setView(m_window->getMazeView());
          sf::View mazeView = m_window->getMazeView();  // TODO: where should we store the mazeView? In config?
          sf::FloatRect mazeBounds = getBoundingRectangle(conf::MazeView);
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
          m_textbox.addString(msg.str());
        }
        if (event.event.type == sf::Event::KeyReleased) {
          if (event.event.key.code == sf::Keyboard::A || event.event.key.code == sf::Keyboard::D) {
            snapped = false;
          }
        }
        break;
      case EventType::USER_EVENT:
        m_textbox.addString("USER AppEvent");
        break;
      default:
        m_textbox.addString("UNHANDLED AppEvent");
        break;
    }
  }

  float snapToNearest45(float angle) {
    return std::round(angle / 45.0f) * 45.0f;  //
  }

  // Function to snap to the next lower multiple of 45 degrees
  float snapToLower45(float angle) {
    return std::floor(angle / 45.0f) * 45.0f;  //
  }
  // Function to/ snap to the next higher multiple of 45 degrees
  float snapToHigher45(float angle) {
    return std::ceil(angle / 45.0f) * 45.0f;  //
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
  void handleInput() {
    /// This is a silly example of how HandleInput can be used
    if (sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
      m_textbox.setBackgroundColour(sf::Color(255, 255, 0, 48));
    } else {
      m_textbox.setBackgroundColour(sf::Color(255, 255, 255, 48));
    }
    float v = 0;
    float w = 0;

    if (!snapped && sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::RControl)) {
        m_robot.setOrientation(snapToHigher45(m_robot.getOrientation() + 1));
        snapped = true;
      } else {
        w = 90;
      }
    }
    if (!snapped && sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::RControl)) {
        m_robot.setOrientation(snapToLower45(m_robot.getOrientation() - 1));
        snapped = true;
      } else {
        w = -90;
      }
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
  void update(sf::Time deltaTime = sf::seconds(0.01)) {
    static bool maze_changed = true;
    m_window->update();  // call this first to process window events
    m_elapsed += deltaTime;
    std::string msg;
    SensorData sensors = m_robot.getSensorData();
    char str[100];
    sprintf(str, "%4d %4d %4d %4d ", (int)sensors.lfs_value, (int)sensors.lds_value, (int)sensors.rds_value, (int)sensors.rfs_value);
    msg += str;
    msg += "\n";
    msg += "sensor update: " + std::to_string(m_process_time.asMicroseconds()) + " us";
    msg += "\n";
    float angle = m_robot.getOrientation();
    sf::Vector2f pos = m_robot.getPose();
    int cell = m_maze_manager.getCellFromPosition(pos.x, pos.y);
    int cell_x = int(pos.x / m_maze_manager.getCellSize());
    int cell_y = int(pos.y / m_maze_manager.getCellSize());
    sprintf(str, "Robot: (%4d,%4d) %4d  cell:%3d = %2d,%2d\n", (int)pos.x, (int)pos.y, (int)angle, cell, cell_x, cell_y);
    msg += str;

    std::vector<int> wall_list = getLocalWallList(cell_x, cell_y);
    for (auto& wall : wall_list) {
      msg += getWallInfo(wall);
    }
    if (snapped) {
      msg += "\nSNAPPED";
    }
    ImGui::SFML::Update(*m_window->getRenderWindow(), m_frame_clock.restart());
    ImGui::Begin("ImGui dialog");
    ImGui::Text("Select the Maze data:");
    if (ImGui::Combo("Maze", &m_maze_index, m_maze_names.data(), (int)m_maze_names.size())) {
      maze_changed = true;
    }
    ImGui::End();
    if (maze_changed) {
      MazeDataSource m = mazeList[m_maze_index];
      if (m.size == 256) {
        m_maze_manager.loadFromMemory(m.data, 16);
      } else {
        m_maze_manager.loadFromMemory(m.data, 32);
      }
      std::string maze_name = m.title;
      maze_name += "(";
      maze_name += std::to_string(m_maze_index);
      maze_name += ")";
      m_txt_maze_name.setString(maze_name);
      maze_changed = false;
    }

    m_adhoc_text.setString(msg);
  }

  std::string getWallInfo(int index) {
    sf::FloatRect r = m_maze_manager.getWallRect(index);
    char str[50];
    sprintf(str, "%5d:  %5d  %5d  %5d  %5d  state:%2d\n", index, (int)r.left, (int)r.top, (int)r.width, (int)r.height, (int)m_maze_manager.getWallState(index));
    return std::string(str);
  }

  std::vector<int> getLocalWallList(int x, int y) {
    std::vector<int> wall_list;
    int w_n = m_maze_manager.getWallIndex(x, y, Direction::North);
    int w_e = m_maze_manager.getWallIndex(x, y, Direction::East);
    int w_s = m_maze_manager.getWallIndex(x, y, Direction::South);
    int w_w = m_maze_manager.getWallIndex(x, y, Direction::West);
    wall_list.push_back(w_n);
    wall_list.push_back(w_e);
    wall_list.push_back(w_s);
    wall_list.push_back(w_w);
    return wall_list;
  }

  std::string showWallIDs() {
    sf::Vector2f pos = m_robot.getPose();
    int cell_x = pos.x / m_maze_manager.getCellSize();
    int cell_y = pos.y / m_maze_manager.getCellSize();
    int x = cell_x;
    int y = cell_y;
    //    for (int x = cell_x - 1; x <= cell_x + 1; x++) {
    //      for (int y = cell_y - 1; y <= cell_y + 1; y++) {
    int w_n = m_maze_manager.getWallIndex(x, y, Direction::North);
    int w_e = m_maze_manager.getWallIndex(x, y, Direction::East);
    int w_s = m_maze_manager.getWallIndex(x, y, Direction::South);
    int w_w = m_maze_manager.getWallIndex(x, y, Direction::West);
    WallState s_n = m_maze_manager.getWallState(w_n);
    WallState s_e = m_maze_manager.getWallState(w_e);
    WallState s_s = m_maze_manager.getWallState(w_s);
    WallState s_w = m_maze_manager.getWallState(w_w);
    char str[50];
    sprintf(str, "%5d (%d) %5d (%d) %5d (%d) %5d (%d)\n", w_n, s_n, w_e, s_e, w_s, s_s, w_w, s_w);
    return std::string(str);
  }

  void drawLidar(sf::RenderTarget& window) {
    float range = conf::SENSOR_MAX_RANGE;
    sf::Vector2f origin = m_robot.getPose();
    sf::FloatRect wall = m_maze_manager.getWallRect(m_maze_manager.getWallIndex(3, 3, Direction::East));
    auto pos = m_robot.getPose();
    auto walls = getLocalWallList(pos.x / m_maze_manager.getCellSize(), pos.y / m_maze_manager.getCellSize());
    for (int a = -179; a < 179; a += 1) {
      float angle = a + m_robot.getOrientation();
      sf::Vector2f ray{cosf((float)angle * RADIANS), sinf((float)angle * RADIANS)};
      float min_d = range;
      for (auto& i : walls) {
        if (m_maze_manager.getWallState(i) == WallState::KnownPresent) {
          sf::FloatRect w = m_maze_manager.getWallRect(i);
          float d = Collisions::getRayDistanceToAlignedRectangle(origin, ray, w, range);
          if (d < min_d) {
            min_d = d;  //
          }
        }
      }
      sf::Vector2f p = origin;
      sf::Vector2f q = origin + ray * min_d;
      //      Drawing::drawLine(window, p, q, sf::Color(255, 255, 255, 32));
      Drawing::drawDot(window, q);
    }
  }
  /// The Render() method is the only place that output is generated for the
  /// window - and any audio devices if used. It is called after the Update()
  /// method and will may be called once per frame or after every update depending
  /// on the underlying update strategy.

  void render() {
    /// ALWAYS do this first
    m_window->beginDraw();
    // grab the window reference to save typing
    sf::RenderWindow& window = *m_window->getRenderWindow();
    window.setView(m_window->getMazeView());
    // Render the physical maze TODO: think about how to add and distinguish the robot map from the physical maze
    m_maze_manager.render(window);
    m_robot_body.draw(window);
    //    drawLidar(window);
    if (conf::DebugHighlightTestedWalls) {
      m_maze_manager.resetPostColours();
      m_maze_manager.resetWallColours();
    }

    window.setView(m_window->getUIView());
    // we can draw anything else we want here.

    m_adhoc_text.setPosition(1000, 200);
    window.draw(m_adhoc_text);

    m_textbox.draw(window);
    window.draw(m_txt_maze_name);

    ImGui::SFML::Render(*m_window->getRenderWindow());
    /// ALWAYS do this last
    m_window->endDraw();
  }

  Window* getWindow() { return m_window.get(); }

  sf::Time getElapsed() { return m_elapsed; }

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
  SensorData callbackCalculateSensorData(float x, float y, float theta) {
    m_timer.restart();
    m_robot_body.updateSensorGeometry(x, y, theta);
    m_obstacles = m_maze_manager.GetObstacles(x, y);
    m_robot_body.updateSensors(m_obstacles);
    m_sensor_data.lfs_value = m_robot_body.getSensor(conf::LFS).getPower();
    m_sensor_data.lds_value = m_robot_body.getSensor(conf::LDS).getPower();
    m_sensor_data.rds_value = m_robot_body.getSensor(conf::RDS).getPower();
    m_sensor_data.rfs_value = m_robot_body.getSensor(conf::RFS).getPower();
    m_process_time = m_timer.getElapsedTime();
    return m_sensor_data;
  }

 private:
  std::unique_ptr<Window> m_window;
  Robot m_robot;  // The robot instance
  RobotBody m_robot_body;
  std::vector<sf::FloatRect> m_obstacles;
  std::vector<const char*> m_maze_names;
  int m_maze_index = 0;
  SensorData m_sensor_data;  // sensor readings we pass back to the robot
  MazeManager m_maze_manager;

  // used for timing code fragments
  sf::Clock m_timer;
  sf::Time m_process_time;
  sf::Time m_elapsed;
  sf::Clock m_frame_clock;

  // use these for adhoc messages, overlays and the like
  sf::Font m_default_font;
  sf::Text m_adhoc_text;
  sf::Text m_txt_maze_name;
  Textbox m_textbox;
  bool snapped = false;
};

#endif  // APPLICATION_H
