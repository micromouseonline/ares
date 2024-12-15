//
// Created by peter on 22/11/24.
//

#ifndef APPLICATION_H
#define APPLICATION_H

#include <SFML/Graphics.hpp>
#include <cstdlib>

#include <iomanip>
#include <iostream>
#include <sstream>
#include "application/event_observer.h"
#include "application/maze-manager.h"
#include "application/robot-wall-sensor.h"
#include "application/textbox.h"
#include "application/window.h"
#include "behaviour/behaviour.h"
#include "common/core.h"
#include "common/vec2.h"
#include "imgui-SFML.h"
#include "imgui.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include "imgui_internal.h"
#pragma GCC diagnostic pop

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
    m_textbox.initialise(10, 14, 400, sf::Vector2f(1000, 10));
    m_textbox.addString("Hello World!");
    m_textbox.addString("WASD keys to move robot");

    m_maze_index = 0;  // Japan2007ef is 48
    /// Have the window inform us of any events
    m_window->addObserver(this);

    if (!ImGui::SFML::Init(*m_window->getRenderWindow())) {
      std::cerr << "ImGui failed to initialise" << std::endl;
    }
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 5.0f;
    ImGuiIO& io = ImGui::GetIO();
    m_guiFont = io.Fonts->AddFontFromFileTTF("assets/fonts/ubuntu-mono-r.ttf", 16);
    (void)ImGui::SFML::UpdateFontTexture();
    io.FontDefault = m_guiFont;

    for (int i = 0; i < mazeCount; i++) {
      m_maze_names.push_back(mazeList[i].title);
    }

    /// set up the robot
    m_robot_body.setRobot(m_robot);
    sf::Vector2f start_pos = m_maze_manager.getCellCentre(0, 0);
    m_robot.setPosition(start_pos.x, start_pos.y);
    m_robot.setOrientation(90.0);
    /// The Lambda expression here serves to bind the callback to the application instance
    m_robot.setSensorCallback([this](float x, float y, float theta) -> SensorData { return this->callbackCalculateSensorData(x, y, theta); });
    m_robot.start();
    m_mouse.setRobot(m_robot);
    m_mouse.start();
  }

  ~Application() {
    m_mouse.stop();
    m_window.reset();  // destroys the window explicitly so that we can clean up
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

    if (!snapped && sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::RControl)) {
        m_robot.setOrientation(snapToHigher45(m_robot.getOrientation() + 1));
        snapped = true;
      }
    }
    if (!snapped && sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::RControl)) {
        m_robot.setOrientation(snapToLower45(m_robot.getOrientation() - 1));
        snapped = true;
      }
    }
  }

  std::string formatSensorData(int lfs, int lds, int rds, int rfs) {
    std::stringstream ss;
    ss << std::setw(5) << lfs << " "    //
       << std::setw(5) << lds << " "    //
       << std::setw(5) << rds << " "    //
       << std::setw(5) << rfs << "\n";  //
    return ss.str();
  }
  std::string formatRobotState() {
    std::stringstream ss;
    sf::Vector2f pos = m_robot.getPose();
    int cell = m_maze_manager.getCellFromPosition(pos.x, pos.y);
    int cell_x = int(pos.x / m_maze_manager.getCellSize());
    int cell_y = int(pos.y / m_maze_manager.getCellSize());
    ss << "Robot:  X: " << (int)pos.x << "\n"                                                           //
       << "        y: " << (int)pos.y << "\n"                                                           //
       << "    theta: " << std::fixed << std::setprecision(1) << m_robot.getOrientation() << " deg \n"  //
       << "\n"                                                                                          //
       << " Cell:  [" << cell_x << ", " << cell_y << "] = " << cell << "\n";                            //
    return ss.str();
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
    //    sf::RenderWindow& window = *m_window->getRenderWindow();
    // sf::Vector2u window_size = window.getSize();
    m_window->update();  // call this first to process window events
    m_elapsed += deltaTime;
    ImGui::SFML::Update(*m_window->getRenderWindow(), m_frame_clock.restart());
    std::stringstream ss;
    SensorData sensors = m_robot.getSensorData();
    ss << "power:  " + formatSensorData((int)sensors.lfs_power, (int)sensors.lds_power, (int)sensors.rds_power, (int)sensors.rfs_power);
    ss << " Dist:  " + formatSensorData((int)sensors.lfs_distance, (int)sensors.lds_distance, (int)sensors.rds_distance, (int)sensors.rfs_distance);
    ss << "\n";
    ss << formatRobotState();
    ss << "\n";
    ss << "sensor update time : " << std::setw(3) << std::to_string(m_process_time.asMicroseconds()) << " us\n";
    m_adhoc_text.setString(ss.str());

    /////  IMGUI ////////////////////////////////////////////////////////////////////////////
    ImGui::SetNextWindowSize(ImVec2(400, 600));
    //    ImGui::SetNextWindowPos(ImVec2(1440, 10));
    bool True = true;
    ImGui::Begin("ImGui dialog", &True, ImGuiWindowFlags_NoResize);
    //    ImGui::PushFont(m_guiFont);
    ImGui::Text("Select the Maze data:");
    if (ImGui::Combo("Maze", &m_maze_index, m_maze_names.data(), (int)m_maze_names.size())) {
      maze_changed = true;
    }

    static int counts = 2;

    ImGui::AlignTextToFramePadding();
    ImGui::Text("Turn Count:");
    ImGui::SameLine();
    float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
    ImGui::PushItemFlag(ImGuiItemFlags_ButtonRepeat, true);
    if (ImGui::ArrowButton("##left", ImGuiDir_Left)) {
      counts--;
    }
    ImGui::SameLine(0.0f, spacing);
    if (ImGui::ArrowButton("##right", ImGuiDir_Right)) {
      counts++;
    }
    counts = std::clamp(counts, 1, 20);
    ImGui::SameLine();
    ImGui::Text("%d", counts);
    ImGui::PopItemFlag();
    float b_wide = ImGui::CalcTextSize("TEST SS180R").x;
    b_wide += ImGui::GetStyle().FramePadding.x * 2.0;
    if (ImGui::Button("TEST SS90", ImVec2(b_wide, 0))) {
      m_robot.resetState();
      m_mouse.go(1, counts);
    }
    if (ImGui::Button("TEST SS180", ImVec2(b_wide, 0))) {
      m_robot.resetState();
      m_mouse.go(2, counts);
    }

    if (ImGui::Button("RESET", ImVec2(b_wide, 0))) {
      m_robot.resetState();
    }
    RobotState state = m_robot.getState();
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "    time     X      Y   Theta     Vel   Omega");
    char s[60];
    sprintf(s, "%8u %5.1f  %5.1f  %6.2f  %6.1f  %6.1f  ", state.timestamp, state.x, state.y, state.theta, state.v, state.omega);
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "%s", s);
    const int frames = 60 * 4;
    static float speed[frames];
    static float omega[frames];
    static float rds[frames];
    static int index = 0;
    speed[index] = state.v;
    omega[index] = state.omega;
    rds[index] = m_robot.getSensorData().rds_power;
    index = (index + 1) % IM_ARRAYSIZE(speed);
    ImGui::PlotLines("speed", speed, IM_ARRAYSIZE(speed), index, "", 0, 3000, ImVec2(0, 100));
    ImGui::PlotLines("omega", omega, IM_ARRAYSIZE(omega), index, "", -500, 500, ImVec2(0, 100));
    ImGui::PlotLines("RDS", rds, IM_ARRAYSIZE(rds), index, "", 00, 500, ImVec2(0, 100));

    //    ImGui::PopFont();
    ImGui::End();
    ImGui::ShowDemoWindow();
    //////////////////////////////////////////////////////////////////////////////////////////

    m_maze_manager.setHighlightObstacles(m_highlight_sensor_region);
    if (maze_changed) {
      MazeDataSource m = mazeList[m_maze_index];
      m_maze_manager.loadFromMemory(m.data, m.size);

      std::string maze_name = m.title;
      maze_name += "(";
      maze_name += std::to_string(m_maze_index);
      maze_name += ")";
      m_txt_maze_name.setString(maze_name);
      maze_changed = false;
    }
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
    sprintf(str, "%5d (%d) %5d (%d) %5d (%d) %5d (%d)\n", w_n, (int)s_n, w_e, (int)s_e, w_s, (int)s_s, w_w, (int)s_w);
    return std::string(str);
  }

  void drawLidar(sf::RenderTarget& window) {
    float range = conf::SENSOR_MAX_RANGE;
    sf::Vector2f origin = m_robot.getPose();
    // sf::FloatRect wall = m_maze_manager.getWallRect(m_maze_manager.getWallIndex(3, 3, Direction::East));
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
      // sf::Vector2f p = origin;
      //      Drawing::drawLine(window, p, q, sf::Color(255, 255, 255, 32));
      sf::Vector2f q = origin + ray * min_d;
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

    // TODO this is sketchy for the highlight. We can do better.
    //      by telling the maze manager what to do
    m_maze_manager.render(window);

    m_robot_body.draw(window);
    //    drawLidar(window);

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

  Window* getWindow() {
    return m_window.get();
  }

  sf::Time getElapsed() {
    return m_elapsed;
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
  SensorData callbackCalculateSensorData(float x, float y, float theta) {
    m_timer.restart();
    m_robot_body.updateSensorGeometry(x, y, theta);
    m_obstacles = m_maze_manager.GetObstacles(x, y);
    m_robot_body.updateSensors(m_obstacles);
    m_sensor_data.lfs_power = m_robot_body.getSensor(conf::LFS).getPower();
    m_sensor_data.lds_power = m_robot_body.getSensor(conf::LDS).getPower();
    m_sensor_data.rds_power = m_robot_body.getSensor(conf::RDS).getPower();
    m_sensor_data.rfs_power = m_robot_body.getSensor(conf::RFS).getPower();

    m_sensor_data.lfs_distance = m_robot_body.getSensor(conf::LFS).getDistance();
    m_sensor_data.lds_distance = m_robot_body.getSensor(conf::LDS).getDistance();
    m_sensor_data.rds_distance = m_robot_body.getSensor(conf::RDS).getDistance();
    m_sensor_data.rfs_distance = m_robot_body.getSensor(conf::RFS).getDistance();
    m_process_time = m_timer.getElapsedTime();
    return m_sensor_data;
  }

 private:
  std::unique_ptr<Window> m_window;
  Behaviour m_mouse;
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
  bool m_highlight_sensor_region = false;
  ImFont* m_guiFont = nullptr;
  ;
};

#endif  // APPLICATION_H
