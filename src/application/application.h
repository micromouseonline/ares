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
#include "event_observer.h"
#include "imgui-SFML.h"
#include "imgui.h"
#include "maze-manager.h"
#include "robot-manager.h"
#include "robot-wall-sensor.h"
#include "vec2.h"
#include "window.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include "imgui_internal.h"
#pragma GCC diagnostic pop
#include "applog-manager.h"
#include "widgets.h"

#include "behaviour/mouse.h"
#include "behaviour/path-printer.h"
#include "robot-body.h"
#include "vehicle/vehicle.h"

const Activity activity[] = {
    ACT_NONE,         ACT_CONTEST,    ACT_SEARCH,     ACT_SPEED_1,    ACT_SPEED_2,   ACT_SPEED_3,    ACT_SPEED_4,   ACT_SPEED_5,    ACT_TEST_FOLLOW_TO,
    ACT_TEST_CIRCUIT, ACT_TEST_SS90E, ACT_TEST_SS90F, ACT_TEST_SS180, ACT_TEST_SD45, ACT_TEST_SD135, ACT_TEST_DS45, ACT_TEST_DS135, ACT_TEST_DD90,
};

class Application : public IEventObserver {
 public:
  Application()
      : m_window(std::make_unique<Window>(conf::AppName, conf::WindowSize)),
        m_vehicle_state(),
        m_vehicle(),
        m_mouse(m_vehicle),
        m_robot_manager(m_mouse) {
    ARES_INFO("APP: Initialising Application ...");
    m_elapsed = sf::Time::Zero;
    setupWindow();
    ARES_TRACE("APP:   .. Window Ready");
    setupImGui();
    ARES_TRACE("APP:   .. ImGui Ready");
    setupVehicle();
    ARES_TRACE("APP:   .. Robot Ready");
    printActionListWithCost(test_path);
  }

  ~Application() {
    ARES_TRACE("APP: Application Shutting Down ...");
    m_robot_manager.resetRobot();
    ARES_TRACE("APP:   .. Robot Manager Stopped");
    m_window.reset();  // destroys the window explicitly so that we can clean up
    ARES_TRACE("APP:   .. Window Closed");
    ImGui::SFML::Shutdown();
    ARES_TRACE("APP:   .. ImGui Shutdown");
  }

  void run() {
    ARES_TRACE("APP: Application running");
    //    m_robot_manager.start();
    while (!getWindow()->isDone()) {
      handleInput();
      update();
      render();
    }
    ARES_TRACE("APP: Application Finished");
  }

  void setupWindow() {
    m_window->addObserver(this);  /// Have the window inform us of any events
    for (int i = 0; i < mazeCount; i++) {
      m_maze_names.push_back(mazeList[i].title);
    }
    m_maze_index = 49;  // Japan2007ef is 49

    m_default_font.loadFromFile("assets/fonts/ubuntu-mono-r.ttf");
    m_txt_maze_name.setCharacterSize(16);
    m_txt_maze_name.setFillColor(sf::Color::Yellow);
    m_txt_maze_name.setPosition(10, 973);
    m_txt_maze_name.setFont(m_default_font);
  }

  void setupImGui() {
    if (!ImGui::SFML::Init(*m_window->getRenderWindow())) {
      ARES_FATAL("APP: Failed to initialise ImGui");
    }

    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 5.0f;

    ImGuiIO& io = ImGui::GetIO();
    m_guiFont = io.Fonts->AddFontFromFileTTF("assets/fonts/ubuntu-mono-r.ttf", 16);
    (void)ImGui::SFML::UpdateFontTexture();
    io.FontDefault = m_guiFont;
  }

  /***
   * The application needs to configure some aspects of the vehicle.
   * Although the vehicle code normally runs in a separate thread, it is
   * safe to do this here because the thread will not have started.
   * Also, we need to do it here because the sensor callback is local.
   */
  void setupVehicle() {
    ARES_INFO("APP: Set Vehicle pose");
    sf::Vector2f start_pos = m_maze_manager.getCellCentre(0, 0);
    m_robot_manager.setVehiclePose(start_pos.x, start_pos.y, 90.0f);

    ARES_INFO("APP: Set Vehicle sensor callback");
    /// The Lambda expression here serves to bind the callback to the application instance
    m_vehicle.setSensorCallback([this](VehicleState state) -> VehicleInputs { return sensorDataCallback(state); });

    ARES_INFO("APP: Vehicle running");
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

  void renderLogMessageWindow() {
    m_robot_manager.processOutputQueue(g_log_messages);  ///                          make the log  vector and use that
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
    int cell_x = int(pos.x / m_maze_manager.getCellSize());
    int cell_y = int(pos.y / m_maze_manager.getCellSize());
    ss << "        X: " << std::setw(5) << (int)pos.x << " Y: " << std::setw(5) << (int)pos.y           //
       << " Theta: " << std::setw(6) << std::fixed << std::setprecision(1) << state.angle << " deg \n"  //
       << " Location:  [" << cell_x << ", " << cell_y << "] = " << "\n";                                //
    return ss.str();
  }

  std::pair<int, int> getRobotLocation(VehicleState& state) {
    sf::Vector2f pos = {state.x, state.y};
    int cell = m_maze_manager.getCellFromPosition(pos.x, pos.y);
    int cell_x = int(pos.x / m_maze_manager.getCellSize());
    int cell_y = int(pos.y / m_maze_manager.getCellSize());
    return std::pair(cell_x, cell_y);
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
    m_window->update();  // call this first to process window events
    m_elapsed += deltaTime;
    ImGui::SFML::Update(*m_window->getRenderWindow(), m_frame_clock.restart());

    updateMaze();

    renderLogMessageWindow();          // ImGui
    renderMouseControlWindow();        // ImGui
    renderApplicationControlWindow();  // ImGui
    m_textbox.render();                // ImGui
  }

  void updateMaze() {
    if (m_maze_changed) {
      setMaze(m_maze_index);
      m_maze_changed = false;
    }
    m_robot_manager.getMazeCopy();
    m_maze_manager.updateFromMap(m_robot_manager.getMap(), m_robot_manager.getMap().getWidth());
    colourMazeCells();
  }

  void colourMazeCells() {
    m_maze_manager.resetCellColours();

    // m_maze_manager.setCellColour(location.first, location.second, sf::Color::Green);
    for (int x = 0; x < 16; x++) {
      for (int y = 0; y < 16; y++) {
        if (m_robot_manager.getMap().has_unknown_walls(x, y)) {
          m_maze_manager.setCellColour(x, y, conf::MazeUnseenColour);
        }
      }
    }
    m_maze_manager.setCellColour(m_robot_manager.getMap().goal().x, m_robot_manager.getMap().goal().y, conf::MazeGoalColour);
  }

  void renderMouseControlWindow() {
    ImGui::Begin("MouseUI", nullptr);
    renderLEDs();
    int item_type = renderActivityChooser();
    if (item_type >= ACT_TEST_CIRCUIT) {
      static int counts = 2;
      ImGui::AlignTextToFramePadding();
      ImGui::Text("Test Count:");
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
    }

    renderResetButton();
    renderPauseButton();
    renderGoButton(item_type);

    ImGui::SliderFloat("Speedup", &m_speed_scale, 0.01, 10.0, "%4.2f");
    m_robot_manager.setRobotSpeedScale(m_speed_scale);

    drawSensorUpdateTime(m_process_time.asMicroseconds());

    renderStateSummary();
    ImGui::End();
  }

  void showProgressBar(float value, float maxValue, sf::Color color = sf::Color(60, 96, 150, 255)) {
    // Normalize the value to the range [0, 1] for the progress bar
    float progress = value / maxValue;
    // Display the progress bar
    char overlay[12];
    snprintf(overlay, sizeof(overlay), "%3.0f", value);
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, color);
    ImGui::ProgressBar(progress, ImVec2(255.0f, 16.0f), overlay);
    ImGui::PopStyleColor(1);
  }

  void renderStateSummary() {
    ImGui::Text("\nPower:");
    SensorData sensors = m_vehicle_state.sensors;
    ImGui::Text("  lfs");
    ImGui::SameLine();
    showProgressBar(sensors.lfs_power, 1024.0f);
    ImGui::Text("  lds");
    ImGui::SameLine();
    showProgressBar(sensors.lds_power, 1024.0f);
    ImGui::Text("  rds");
    ImGui::SameLine();
    showProgressBar(sensors.rds_power, 1024.0f);
    ImGui::Text("  rfs");
    ImGui::SameLine();
    showProgressBar(sensors.rfs_power, 1024.0f);
    ImGui::Text("\nDistance:");
    ImGui::Text("  lfs");
    ImGui::SameLine();
    showProgressBar(sensors.lfs_distance, 300.0f);
    ImGui::Text("  lds");
    ImGui::SameLine();
    showProgressBar(sensors.lds_distance, 300.0f);
    ImGui::Text("  rds");
    ImGui::SameLine();
    showProgressBar(sensors.rds_distance, 300.0f);
    ImGui::Text("  rfs");
    ImGui::SameLine();
    showProgressBar(sensors.rfs_distance, 300.0f);
    ImGui::NewLine();
    std::stringstream state_summary;
    state_summary << formatRobotState(m_vehicle_state);
    ImGui::Text("%s", state_summary.str().c_str());
  }

  void renderLEDs() const {
    const uint8_t leds = m_vehicle_state.leds;
    for (int i = 7; i >= 0; i--) {
      bool bitState = leds & BIT(i);
      DrawLEDx(bitState, 6, IM_COL32(255, 64, 64, 255));
    }
    ImGui::Text("LEDS");
  }

  void renderResetButton() {
    if (ImGui::Button("RESET", ImVec2(81, 24))) {
      m_robot_buttons |= (uint8_t)BTN_RESET;
      m_paused = false;
      m_robot_manager.resumeRobot();
      m_robot_manager.resetRobot();
      m_maze_changed = true;
    } else {
      m_robot_buttons &= ~(uint8_t)BTN_RESET;
    }
  }

  void renderPauseButton() {
    ImGui::SameLine();

    bool is_paused = m_paused;
    if (is_paused) {
      ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));         // Red
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1.0f, 0.2f, 0.2f, 1.0f));  // Lighter red when hovered
      ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.8f, 0.0f, 0.0f, 1.0f));   // Darker red when active
    }
    if (ImGui::Button(m_paused ? "RESUME" : "PAUSE", ImVec2(82, 24))) {
      m_paused = !m_paused;
      if (m_paused) {
        m_robot_manager.pauseRobot();
      } else {
        m_robot_manager.resumeRobot();
      }
    }
    if (is_paused) {
      ImGui::PopStyleColor(3);
    }
  }

  void renderGoButton(int item_type) {
    ImGui::SameLine();
    if (ImGui::Button("GO", ImVec2(81, 24))) {
      m_robot_manager.setRobotActivity(activity[item_type]);
      m_robot_buttons |= (uint8_t)BTN_GO;
    } else {
      m_robot_buttons &= ~(uint8_t)BTN_GO;
    }
  }

  int renderActivityChooser() {
    const char* item_names[] = {
        "NOTHING",     "Run Contest", "Search", "Speedrun 1", "Speedrun 2", "Speedrun 3", "Speedrun 4", "Speedrun 5", "Wall Follower",
        "Circuit Run", "SS90E",       "SS90F",  "SS180",      "SD45",       "SD135",      "DS45",       "DS135",      "DD90",
    };
    static int item_type = 2;
    ImGui::Combo("Item Type", &item_type, item_names, IM_ARRAYSIZE(item_names), IM_ARRAYSIZE(item_names));
    return item_type;
  }

  void resetRobot() {
    m_robot_manager.resumeRobot();
    m_robot_manager.resetRobot();
    m_maze_changed = true;
  }
  void renderApplicationControlWindow() {  /////  IMGUI ////////////////////////////////////////////////////////////////////////////
    ImGui::Begin("Application Control", nullptr);
    ImGui::Text("Select the Maze data:");
    if (ImGui::Combo("Maze", &m_maze_index, m_maze_names.data(), (int)m_maze_names.size())) {
      m_maze_changed = true;
    }

    /// TODO: the log level should be passed in through the vehicle state - like a button push or toggle
    bool makes_detailed_event_log = m_robot_manager.isRobotEventLogDetailed();
    if (ImGui::Checkbox("Show Detailed Mouse Event Log", &makes_detailed_event_log)) {
      m_robot_manager.setRobotEventLogDetailed(makes_detailed_event_log);
    }

    static bool continuous_search = true;
    if (ImGui::Checkbox("Continuous Search", &continuous_search)) {
      m_robot_manager.setRobotContinuousMode(continuous_search);
    }
    renderStateData();
    ImGui::End();
  }

  void setMaze(int index) {
    MazeDataSource m = mazeList[index];
    m_maze_manager.loadFromMemory(m.data, m.size);
    std::string maze_name = m.title;
    maze_name += "(" + std::to_string(index) + ")";
    m_txt_maze_name.setString(maze_name);
    m_robot_manager.resetRobot();
    m_robot_manager.initRobot();
  }

  void renderStateData() const {
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "    time     X      Y   Theta     Vel   Omega");
    char s[60];
    /// NOTE: if the tick count is increasing, updateMotion is running and the thread is active
    sprintf(s, "%8u %5.1f  %5.1f  %6.2f  %6.1f  %6.1f  ",  //
            m_vehicle_state.ticks, m_vehicle_state.x, m_vehicle_state.y, m_vehicle_state.angle, m_vehicle_state.velocity, m_vehicle_state.angular_velocity);
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "%s", s);
    const int frames = 60 * 4;
    static int index = 0;
    static float speed[frames];
    static float omega[frames];
    static float rds[frames];
    speed[index] = m_vehicle_state.velocity;
    omega[index] = m_vehicle_state.angular_velocity;
    rds[index] = m_vehicle_state.sensors.rds_power;
    index = (index + 1) % IM_ARRAYSIZE(speed);
    ImGui::PlotLines("speed", speed, IM_ARRAYSIZE(speed), index, "", 0, 3000, ImVec2(330, 100));
    ImGui::PlotLines("angular_velocity", omega, IM_ARRAYSIZE(omega), index, "", -1000, 1000, ImVec2(330, 140));
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
    for (int x = 0; x < 16; x++) {
      for (int y = 0; y < 16; y++) {
        // Wall wall = m_robot_manager.maze().walls[x][y];
        // m_maze_manager.set
      }
    }
    // TODO this is sketchy for the highlight. We can do better.
    //      by telling the maze manager what to do
    m_maze_manager.render(window);
    m_robot_body.draw(window, m_vehicle_state.x, m_vehicle_state.y, m_vehicle_state.angle);

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
    m_vehicle_inputs.sensors.lfs_power = m_robot_body.getSensor(conf::LFS).getPower();
    m_vehicle_inputs.sensors.lds_power = m_robot_body.getSensor(conf::LDS).getPower();
    m_vehicle_inputs.sensors.rds_power = m_robot_body.getSensor(conf::RDS).getPower();
    m_vehicle_inputs.sensors.rfs_power = m_robot_body.getSensor(conf::RFS).getPower();

    m_vehicle_inputs.sensors.lfs_distance = m_robot_body.getSensor(conf::LFS).getDistance();
    m_vehicle_inputs.sensors.lds_distance = m_robot_body.getSensor(conf::LDS).getDistance();
    m_vehicle_inputs.sensors.rds_distance = m_robot_body.getSensor(conf::RDS).getDistance();
    m_vehicle_inputs.sensors.rfs_distance = m_robot_body.getSensor(conf::RFS).getDistance();

    m_vehicle_inputs.buttons = m_robot_buttons;
    m_process_time = m_timer.getElapsedTime();
    /// the returned data is copied so there is no need for a lock
    return m_vehicle_inputs;
  }

 private:
  std::unique_ptr<Window> m_window;

  VehicleState m_vehicle_state;
  Vehicle m_vehicle;  // The robot instance
  Mouse m_mouse;
  RobotManager m_robot_manager;

  RobotBody m_robot_body;
  std::vector<sf::FloatRect> m_obstacles;
  std::vector<const char*> m_maze_names;
  int m_maze_index = 0;
  bool m_maze_changed = true;
  bool m_paused = false;
  float m_speed_scale = 1.0f;
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
  uint8_t m_robot_buttons = 0;
};

#endif  // APPLICATION_H
