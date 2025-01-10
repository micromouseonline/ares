//
// Created by peter on 22/11/24.
//

#ifndef APPLICATION_H
#define APPLICATION_H

#include <SFML/Graphics.hpp>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include "common/core.h"
#include "common/vec2.h"
#include "event_observer.h"
#include "imgui-SFML.h"
#include "imgui.h"
#include "maze-manager.h"
#include "robot-manager.h"
#include "robot-wall-sensor.h"
#include "window.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include "imgui_internal.h"
#pragma GCC diagnostic pop
#include "common/logmanager.h"
#include "widgets.h"

#include "behaviour/mouse.h"
#include "robot-body.h"
#include "vehicle/sensor-data.h"
#include "vehicle/vehicle.h"

class Application : public IEventObserver {
 public:
  Application()
      : m_window(std::make_unique<Window>(conf::AppName, conf::WindowSize)),  //
        m_behaviour(m_mouse, m_robot) {                                       //

    ARES_INFO("Initialising Application");
    m_elapsed = sf::Time::Zero;
    setupWindow();
    ARES_TRACE("  .. Window Ready");
    setupImGui();
    ARES_TRACE("  .. ImGui Ready");
    setupRobot();
    ARES_TRACE("  .. Robot Ready");
  }

  ~Application() {
    ARES_TRACE("Application Shutting Down ...");
    m_mouse.stop();
    ARES_TRACE("   Mouse Stopped");
    m_window.reset();  // destroys the window explicitly so that we can clean up
    ARES_TRACE("  .. Window Closed");
    ImGui::SFML::Shutdown();
    ARES_TRACE("  .. ImGui Shutdown");
  }

  void run() {
    while (!getWindow()->isDone()) {
      handleInput();
      update();
      render();
    }
  }

  void setupWindow() {
    m_window->addObserver(this);  /// Have the window inform us of any events
    for (int i = 0; i < mazeCount; i++) {
      m_maze_names.push_back(mazeList[i].title);
    }
    m_maze_index = 48;  // Japan2007ef is 48

    m_default_font.loadFromFile("assets/fonts/ubuntu-mono-r.ttf");
    m_txt_maze_name.setCharacterSize(16);
    m_txt_maze_name.setFillColor(sf::Color::Yellow);
    m_txt_maze_name.setPosition(10, 973);
    m_txt_maze_name.setFont(m_default_font);
  }

  void setupImGui() {
    if (!ImGui::SFML::Init(*m_window->getRenderWindow())) {
      ARES_FATAL("Failed to initialise ImGui");
    }

    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 5.0f;

    ImGuiIO& io = ImGui::GetIO();
    m_guiFont = io.Fonts->AddFontFromFileTTF("assets/fonts/ubuntu-mono-r.ttf", 16);
    (void)ImGui::SFML::UpdateFontTexture();
    io.FontDefault = m_guiFont;
  }

  void setupRobot() {
    m_robot_body.setRobot(m_robot);
    sf::Vector2f start_pos = m_maze_manager.getCellCentre(0, 0);
    m_robot.setPose(start_pos.x, start_pos.y, 90.0f);
    /// The Lambda expression here serves to bind the callback to the application instance
    m_robot.setSensorCallback([this](VehicleState state) -> VehicleInputs { return sensorDataCallback(state); });
    m_robot.start();
    m_mouse.setVehicle(m_robot);
    m_mouse.start();
    m_behaviour.start();
  }

  /***
   * OnEvent is used to respond to CHANGES in state such as keypress
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

  void handleInput() {
    /// This is not events. Test keyboard or mouse button STATE here
  }

  void displayLogMessages() {
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

  std::string formatRobotState(VehicleState& state) {
    std::stringstream ss;
    sf::Vector2f pos = {state.x, state.y};
    int cell = m_maze_manager.getCellFromPosition(pos.x, pos.y);
    int cell_x = int(pos.x / m_maze_manager.getCellSize());
    int cell_y = int(pos.y / m_maze_manager.getCellSize());
    ss << "Vehicle:  X: " << (int)pos.x << "\n"                                            //
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

    /// TODO: this needs sorting out. It may not be thread-safe
    ///       Is it better to just listen for mapping messages from the mouse?
    m_maze_manager.updateFromMap(m_mouse.getMaze(), m_mouse.getMaze().getWidth());

    //    m_vehicle_state = m_robot.getState();
    std::stringstream state_summary;
    VehicleInputs sensors = m_vehicle_state.vehicle_inputs;
    state_summary << "power:  " + formatSensorData((int)sensors.lfs_power, (int)sensors.lds_power, (int)sensors.rds_power, (int)sensors.rfs_power);
    state_summary << " Dist:  " + formatSensorData((int)sensors.lfs_distance, (int)sensors.lds_distance, (int)sensors.rds_distance, (int)sensors.rfs_distance);
    state_summary << "\n";
    state_summary << formatRobotState(m_vehicle_state);

    /////  IMGUI ////////////////////////////////////////////////////////////////////////////
    ImGui::Begin("MouseUI", nullptr);
    const uint8_t leds = m_vehicle_state.leds;
    for (int i = 7; i >= 0; i--) {
      bool bitState = leds & BIT(i);
      DrawLED(bitState, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));  // Green color for ON state
      ImGui::SameLine();
    }
    ImGui::Text("LEDS");

    bool button_x = CustomButton("X", ImVec2(104, 24));
    ImGui::SameLine();
    bool button_y = CustomButton("Y", ImVec2(104, 24));
    drawSensorUpdateTime(m_process_time.asMicroseconds());
    ImGui::Text("%s", state_summary.str().c_str());
    ImGui::End();

    /////  IMGUI ////////////////////////////////////////////////////////////////////////////
    ImGui::Begin("Mouse Control", nullptr);
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
      //      m_robot.setPose(start_pos.x, start_pos.y, 90.0f);
      //      m_robot.setSpeeds(0.0f, 0.0f);
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
      m_mouse.setFirstRunState(true);
      m_mouse.go(ACT_TEST_SEARCH, 0);
    }

    bool detailed_event_log = m_mouse.getEventLogDetailed();
    if (ImGui::Checkbox("Show Detailed Mouse Event Log", &detailed_event_log)) {
      m_mouse.setEventLogDetailed(detailed_event_log);
    }

    static bool continuous_search = true;
    if (ImGui::Checkbox("Continuous Search", &continuous_search)) {
      m_mouse.setContinuous(continuous_search);
    }

    static float speedup = 1.0f;
    ImGui::SliderFloat("Speedup", &speedup, 0.25, 4.0, "%4.2f");
    m_mouse.setSpeedUp(speedup);
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "    time     X      Y   Theta     Vel   Omega");
    char s[60];
    sprintf(s, "%8u %5.1f  %5.1f  %6.2f  %6.1f  %6.1f  ",  //
            m_vehicle_state.timestamp, m_vehicle_state.x, m_vehicle_state.y, m_vehicle_state.angle, m_vehicle_state.velocity, m_vehicle_state.omega);
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "%s", s);
    const int frames = 60 * 4;
    static float speed[frames];
    static float omega[frames];
    static float rds[frames];
    static int index = 0;
    speed[index] = m_vehicle_state.velocity;
    omega[index] = m_vehicle_state.omega;
    rds[index] = m_vehicle_state.vehicle_inputs.rds_power;
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
   * We also take the opportunity to return other inputs and state
   * information. Things like current action, button presses...
   * For a better physics simulation, that might include things like
   * mocked IMU readings, battery voltages, downforce...
   *
   * @return a copy of the local input data
   */
  VehicleInputs sensorDataCallback(VehicleState state) {
    m_timer.restart();
    {
      /// ensure we get a clean, complete copy of the state
      std::lock_guard<std::mutex> lock(m_application_mutex);
      m_vehicle_state = state;
    }
    m_robot_body.updateSensorGeometry(m_vehicle_state.x, m_vehicle_state.y, m_vehicle_state.angle);
    m_obstacles = m_maze_manager.GetObstacles(m_vehicle_state.x, m_vehicle_state.y);
    m_robot_body.updateSensors(m_obstacles);
    m_vehicle_inputs.lfs_power = m_robot_body.getSensor(conf::LFS).getPower();
    m_vehicle_inputs.lds_power = m_robot_body.getSensor(conf::LDS).getPower();
    m_vehicle_inputs.rds_power = m_robot_body.getSensor(conf::RDS).getPower();
    m_vehicle_inputs.rfs_power = m_robot_body.getSensor(conf::RFS).getPower();

    m_vehicle_inputs.lfs_distance = m_robot_body.getSensor(conf::LFS).getDistance();
    m_vehicle_inputs.lds_distance = m_robot_body.getSensor(conf::LDS).getDistance();
    m_vehicle_inputs.rds_distance = m_robot_body.getSensor(conf::RDS).getDistance();
    m_vehicle_inputs.rfs_distance = m_robot_body.getSensor(conf::RFS).getDistance();

    m_process_time = m_timer.getElapsedTime();
    /// the returned data is copied so there is no need for a lock
    return m_vehicle_inputs;
  }

 private:
  std::unique_ptr<Window> m_window;

  Mouse m_mouse;
  Vehicle m_robot;  // The robot instance
  VehicleState m_vehicle_state;
  RobotManager m_behaviour;

  RobotBody m_robot_body;
  std::vector<sf::FloatRect> m_obstacles;
  std::vector<const char*> m_maze_names;
  int m_maze_index = 0;
  VehicleInputs m_vehicle_inputs;  // data we pass back to the robot
  MazeManager m_maze_manager;

  // used for timing code fragments
  sf::Clock m_timer;
  sf::Time m_process_time;
  sf::Time m_elapsed;
  sf::Clock m_frame_clock;

  sf::Font m_default_font;
  sf::Text m_txt_maze_name;
  TextBox m_textbox;
  ImFont* m_guiFont = nullptr;
  std::mutex m_application_mutex;
};

#endif  // APPLICATION_H
