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
#include "robot-body.h"
#include "vehicle/vehicle.h"

class Application : public IEventObserver {
 public:
  Application()
      : m_window(std::make_unique<Window>(conf::AppName, conf::WindowSize)), m_vehicle_state(), m_vehicle(), m_mouse(m_vehicle), m_robot_manager(m_mouse) {  //

    ARES_INFO("APP: Initialising Application ...");
    m_elapsed = sf::Time::Zero;
    setupWindow();
    ARES_TRACE("APP:   .. Window Ready");
    setupImGui();
    ARES_TRACE("APP:   .. ImGui Ready");
    setupVehicle();
    ARES_TRACE("APP:   .. Robot Ready");
  }

  ~Application() {
    ARES_TRACE("APP: Application Shutting Down ...");
    //    m_robot_manager.stop();
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
    m_maze_index = 48;  // Japan2007ef is 48

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

  void displayLogMessages() {
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

    std::stringstream state_summary;
    SensorData sensors = m_vehicle_state.sensors;
    state_summary << "power:  " + formatSensorData((int)sensors.lfs_power, (int)sensors.lds_power, (int)sensors.rds_power, (int)sensors.rfs_power);
    state_summary << " Dist:  " + formatSensorData((int)sensors.lfs_distance, (int)sensors.lds_distance, (int)sensors.rds_distance, (int)sensors.rfs_distance);
    state_summary << "\n";
    state_summary << formatRobotState(m_vehicle_state);

    /////  IMGUI ////////////////////////////////////////////////////////////////////////////
    ImGui::Begin("MouseUI", nullptr);
    const uint8_t leds = m_vehicle_state.leds;
    for (int i = 7; i >= 0; i--) {
      bool bitState = leds & BIT(i);
      DrawLEDx(bitState, 6, IM_COL32(255, 64, 64, 255));
    }
    ImGui::Text("LEDS");
    const char* item_names[] = {
        "NOTHING",     "Run Contest", "Search", "Speedrun 1", "Speedrun 2", "Speedrun 3", "Speedrun 4", "Speedrun 5", "Wall Follower",
        "Circuit Run", "SS90E",       "SS90F",  "SS180",      "SD45",       "SD135",      "DS45",       "DS135",      "DD90",
    };
    const Activity activity[] = {
        ACT_NONE,         ACT_CONTEST,    ACT_SEARCH,     ACT_SPEED_1,    ACT_SPEED_2,   ACT_SPEED_3,    ACT_SPEED_4,   ACT_SPEED_5,    ACT_TEST_FOLLOW_TO,
        ACT_TEST_CIRCUIT, ACT_TEST_SS90E, ACT_TEST_SS90F, ACT_TEST_SS180, ACT_TEST_SD45, ACT_TEST_SD135, ACT_TEST_DS45, ACT_TEST_DS135, ACT_TEST_DD90,
    };

    static int item_type = 2;
    ImGui::Combo("Item Type", &item_type, item_names, IM_ARRAYSIZE(item_names), IM_ARRAYSIZE(item_names));

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
    if (ImGui::Button("RESET", ImVec2(81, 24))) {
      m_vehicle_buttons |= (uint8_t)Button::BTN_RESET;
      m_robot_manager.initRobot();
      maze_changed = true;
    } else {
      m_vehicle_buttons &= ~(uint8_t)Button::BTN_RESET;
    }

    ImGui::SameLine();
    static bool paused;
    if (ImGui::Button(paused ? "RESUME" : "PAUSE", ImVec2(82, 24))) {
      paused = !paused;
      if (paused) {
        m_robot_manager.pauseRobot();
        //        manager.pauseCV.notify_all();  // Notify Manager to pause
      } else {
        m_robot_manager.resumeRobot();
        //        manager.pauseCV.notify_all();  // Notify Manager to resume
      }
    }

    ImGui::SameLine();
    if (ImGui::Button("GO", ImVec2(81, 24))) {
      m_mouse.setActivity(activity[item_type]);
      m_vehicle_buttons |= (uint8_t)Button::BTN_GO;
    } else {
      m_vehicle_buttons &= ~(uint8_t)Button::BTN_GO;
    }
    static float speedup = 1.0f;
    ImGui::SliderFloat("Speedup", &speedup, 0.1, 4.0, "%4.2f");
    m_mouse.setSpeedUp(speedup);

    drawSensorUpdateTime(m_process_time.asMicroseconds());
    ImGui::Text("%s", state_summary.str().c_str());
    ImGui::End();

    /////  IMGUI ////////////////////////////////////////////////////////////////////////////
    ImGui::Begin("Mouse Control", nullptr);
    ImGui::Text("Select the Maze data:");
    if (ImGui::Combo("Maze", &m_maze_index, m_maze_names.data(), (int)m_maze_names.size())) {
      maze_changed = true;
    }

    /// TODO: the log level should be passed in through the vehicle state - like a button push or toggle
    bool detailed_event_log = m_mouse.getEventLogDetailed();
    if (ImGui::Checkbox("Show Detailed Mouse Event Log", &detailed_event_log)) {
      m_mouse.setEventLogDetailed(detailed_event_log);
    }

    static bool continuous_search = true;
    if (ImGui::Checkbox("Continuous Search", &continuous_search)) {
      m_mouse.setContinuous(continuous_search);
    }

    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "    time     X      Y   Theta     Vel   Omega");
    char s[60];
    /// NOTE: if the tick count is increasing, updateMotion is running and the thread is active
    sprintf(s, "%8u %5.1f  %5.1f  %6.2f  %6.1f  %6.1f  ",  //
            m_vehicle_state.ticks, m_vehicle_state.x, m_vehicle_state.y, m_vehicle_state.angle, m_vehicle_state.velocity, m_vehicle_state.angular_velocity);
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "%s", s);
    const int frames = 60 * 4;
    static float speed[frames];
    static float omega[frames];
    static float rds[frames];
    static int index = 0;
    speed[index] = m_vehicle_state.velocity;
    omega[index] = m_vehicle_state.angular_velocity;
    rds[index] = m_vehicle_state.sensors.rds_power;
    index = (index + 1) % IM_ARRAYSIZE(speed);
    ImGui::PlotLines("speed", speed, IM_ARRAYSIZE(speed), index, "", 0, 3000, ImVec2(330, 100));
    ImGui::PlotLines("angular_velocity", omega, IM_ARRAYSIZE(omega), index, "", -1000, 1000, ImVec2(330, 140));
    ImGui::PlotLines("RDS", rds, IM_ARRAYSIZE(rds), index, "", -1000, 1000, ImVec2(330, 140));

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
      m_robot_manager.initRobot();
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

    m_vehicle_inputs.buttons = m_vehicle_buttons;
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
  uint8_t m_vehicle_buttons = 0;
};

#endif  // APPLICATION_H
