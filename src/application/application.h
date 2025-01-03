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
#include "behaviour/behaviour.h"
#include "common/core.h"
#include "common/vec2.h"
#include "event_observer.h"
#include "imgui-SFML.h"
#include "imgui.h"
#include "maze-manager.h"
#include "robot-wall-sensor.h"
#include "window.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include "imgui_internal.h"
#pragma GCC diagnostic pop
#include "common/logmanager.h"
#include "robot-body.h"
#include "vehicle/sensor-data.h"
#include "vehicle/vehicle.h"
#include "widgets.h"

class Application : public IEventObserver {
 public:
  Application() : m_window(std::make_unique<Window>(conf::AppName, conf::WindowSize)), m_elapsed(sf::Time::Zero) {
    m_logger.initialise();
    ARES_INFO("Initialising");
    m_elapsed = sf::Time::Zero;

    m_default_font.loadFromFile("assets/fonts/ubuntu-mono-r.ttf");

    m_adhoc_text.setFont(m_default_font);
    m_adhoc_text.setCharacterSize(16);
    m_adhoc_text.setFillColor(sf::Color::Yellow);
    m_txt_maze_name.setFont(m_default_font);
    m_txt_maze_name.setCharacterSize(16);
    m_txt_maze_name.setFillColor(sf::Color::Yellow);
    m_txt_maze_name.setPosition(10, 973);

    m_textbox.addText("Hello World");

    m_maze_index = 48;  // Japan2007ef is 48
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
    m_robot.setPose(start_pos.x, start_pos.y, 90.0f);
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
    m_logger.Shutdown();
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
          window->setView(m_window->getMazeView());
        }
        if (event.event.type == sf::Event::KeyReleased) {
          if (event.event.key.code == sf::Keyboard::A || event.event.key.code == sf::Keyboard::D) {
          }
        }
        break;
      case EventType::USER_EVENT:
        m_textbox.addText("USER AppEvent");
        break;
      default:
        m_textbox.addText("UNHANDLED AppEvent");
        break;
    }
  }

  /***
   * This is not about the events. We can test the state of the mouse and
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
  }

  void displayLogMessages() {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    while (!g_log_messages.empty()) {
      m_textbox.addText(g_log_messages.front());
      g_log_messages.pop();
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

  std::string formatRobotState(RobotState& state) {
    std::stringstream ss;
    sf::Vector2f pos = {state.x, state.y};
    int cell = m_maze_manager.getCellFromPosition(pos.x, pos.y);
    int cell_x = int(pos.x / m_maze_manager.getCellSize());
    int cell_y = int(pos.y / m_maze_manager.getCellSize());
    ss << "Vehicle:  X: " << (int)pos.x << "\n"                                              //
       << "        y: " << (int)pos.y << "\n"                                              //
       << "    angle: " << std::fixed << std::setprecision(1) << state.angle << " deg \n"  //
       << "\n"                                                                             //
       << " Cell:  [" << cell_x << ", " << cell_y << "] = " << cell << "\n";               //
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
    m_window->update();  // call this first to process window events
    m_elapsed += deltaTime;
    ImGui::SFML::Update(*m_window->getRenderWindow(), m_frame_clock.restart());
    displayLogMessages();

    RobotState robot_state = m_robot.getState();
    m_maze_manager.updateFromMap(m_mouse.getMaze(), 16);

    std::stringstream ss;
    SensorData sensors = robot_state.sensor_data;
    ss << "power:  " + formatSensorData((int)sensors.lfs_power, (int)sensors.lds_power, (int)sensors.rds_power, (int)sensors.rfs_power);
    ss << " Dist:  " + formatSensorData((int)sensors.lfs_distance, (int)sensors.lds_distance, (int)sensors.rds_distance, (int)sensors.rfs_distance);
    ss << "\n";
    ss << formatRobotState(robot_state);

    /////  IMGUI ////////////////////////////////////////////////////////////////////////////
    ImGui::Begin("MouseUI", nullptr);
    const uint8_t leds = robot_state.leds;
    const uint8_t buttons = robot_state.buttons;
    for (int i = 7; i >= 0; i--) {
      bool bitState = leds & BIT(i);
      DrawLED(bitState, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));  // Green color for ON state
      ImGui::SameLine();
    }
    ImGui::Text("LEDS");
    for (int i = 7; i >= 0; i--) {
      bool bitState = buttons & BIT(i);
      DrawLED(bitState, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));  // Green color for ON state
      ImGui::SameLine();
    }
    ImGui::Text("BUTTONS");
    m_robot.setButton(1, CustomButton("X", ImVec2(104, 24)));
    ImGui::SameLine();
    m_robot.setButton(0, CustomButton("Y", ImVec2(104, 24)));
    int sensor_update_time = m_process_time.asMicroseconds();
    static int update_time = sensor_update_time;
    float alpha = 0.025;

    update_time = alpha * (sensor_update_time) + (1 - alpha) * update_time;
    static int peak_time = 0;
    if (sensor_update_time > peak_time) {
      peak_time = sensor_update_time;
    } else {
      peak_time = 0.98 * peak_time;
    }
    ImGui::Text("Sensor update %3d us", update_time);
    ImGui::SameLine();
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddRect(p, ImVec2(p.x + 200, p.y + 20), IM_COL32(255, 200, 0, 255));
    ImColor bar_color = IM_COL32(0, 255, 0, 128);
    p.x += 1;
    p.y += 1;
    ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x + std::min(198, peak_time), p.y + 18), IM_COL32(200, 0, 0, 128));
    ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x + update_time, p.y + 18), bar_color);
    ImGui::NewLine();
    ImGui::Text("%s", ss.str().c_str());
    ImGui::End();

    /////  IMGUI ////////////////////////////////////////////////////////////////////////////
    ImGui::Begin("Behaviour Control", nullptr);
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
    ImGui::PopItemFlag();
    counts = std::clamp(counts, 1, 20);
    ImGui::SameLine();
    ImGui::Text("%d", counts);
    float b_wide = ImGui::CalcTextSize("TEST SS180R").x;
    b_wide += ImGui::GetStyle().FramePadding.x * 2.0;
    sf::Vector2f start_pos = m_maze_manager.getCellCentre(0, 0);
    if (ImGui::Button("TEST SS90", ImVec2(b_wide, 0))) {
      m_robot.setPose(start_pos.x, start_pos.y, 90.0f);
      m_mouse.go(ACT_TEST_SS90, counts);
    }
    ImGui::SameLine();
    if (ImGui::Button("TEST SS180", ImVec2(b_wide, 0))) {
      m_robot.setPose(start_pos.x, start_pos.y, 90.0f);
      m_mouse.go(ACT_TEST_SS180, counts);
    }
    ImGui::SameLine();
    if (ImGui::Button("CIRCUIT RUN", ImVec2(b_wide, 0))) {
      m_robot.setPose(start_pos.x, start_pos.y, 90.0f);
      m_mouse.go(ACT_TEST_CIRCUIT, counts);
    }
    if (ImGui::Button("RESET", ImVec2(b_wide, 0))) {
      m_robot.setPose(start_pos.x, start_pos.y, 90.0f);
      m_robot.setSpeeds(0.0f, 0.0f);
      m_mouse.reset();
      maze_changed = true;
      g_ticks = 0;
    }
    ImGui::SameLine();
    if (ImGui::Button("FOLLOWER", ImVec2(b_wide, 0))) {
      m_robot.setPose(start_pos.x, start_pos.y, 90.0f);
      m_mouse.go(ACT_TEST_FOLLOW_TO, 0);
    }
    ImGui::SameLine();
    if (ImGui::Button("SEARCHER", ImVec2(b_wide, 0))) {
      m_robot.setPose(start_pos.x, start_pos.y, 90.0f);
      m_mouse.go(ACT_TEST_SEARCH, 0);
    }
    bool detailed_event_log = m_mouse.getEventLogDetailed();
    if (ImGui::Checkbox("Show Detailed Behaviour Event Log", &detailed_event_log)) {
      m_mouse.setEventLogDetailed(detailed_event_log);
    }
    static float speedup = 1.0f;
    ImGui::SliderFloat("Speedup", &speedup, 0.25, 4.0, "%4.2f");
    m_mouse.setSpeedUp(speedup);
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "    time     X      Y   Theta     Vel   Omega");
    char s[60];
    sprintf(s, "%8u %5.1f  %5.1f  %6.2f  %6.1f  %6.1f  ",  //
            robot_state.timestamp, robot_state.x, robot_state.y, robot_state.angle, robot_state.velocity, robot_state.omega);
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "%s", s);
    const int frames = 60 * 4;
    static float speed[frames];
    static float omega[frames];
    static float rds[frames];

    static int index = 0;
    speed[index] = robot_state.velocity;
    omega[index] = robot_state.omega;
    rds[index] = robot_state.sensor_data.rds_power;
    index = (index + 1) % IM_ARRAYSIZE(speed);
    ImGui::PlotLines("speed", speed, IM_ARRAYSIZE(speed), index, "", 0, 3000, ImVec2(330, 100));
    ImGui::PlotLines("omega", omega, IM_ARRAYSIZE(omega), index, "", -1000, 1000, ImVec2(330, 140));
    ImGui::PlotLines("RDS", rds, IM_ARRAYSIZE(rds), index, "", -1000, 1000, ImVec2(330, 140));

    //    ImGui::PopFont();
    ImGui::End();
    /////  IMGUI ////////////////////////////////////////////////////////////////////////////

    m_textbox.render();

    ImGui::ShowDemoWindow();
    //////////////////////////////////////////////////////////////////////////////////////////

    if (maze_changed) {
      MazeDataSource m = mazeList[m_maze_index];
      m_maze_manager.loadFromMemory(m.data, m.size);

      std::string maze_name = m.title;
      maze_name += "(";
      maze_name += std::to_string(m_maze_index);
      maze_name += ")";
      m_txt_maze_name.setString(maze_name);
      m_mouse.reset();

      m_robot.setPose(start_pos.x, start_pos.y, 90.0f);
      maze_changed = false;
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

    window.setView(m_window->getUIView());
    // we can draw anything else we want here.

    m_adhoc_text.setPosition(1000, 200);
    window.draw(m_adhoc_text);

    window.draw(m_txt_maze_name);

    ImGui::SFML::Render(*m_window->getRenderWindow());
    /// ALWAYS do this last
    m_window->endDraw();
  }

  Window* getWindow() {
    return m_window.get();
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
  Vehicle m_robot;  // The robot instance
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
  TextBox m_textbox;
  ImFont* m_guiFont = nullptr;
  LogManager m_logger;
};

#endif  // APPLICATION_H
