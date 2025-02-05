#include <SFML/Graphics.hpp>
#include "application//applog-manager.h"
#include "application/application.h"

#include "vehicle/hal/board-ares.h"
#include "vehicle/vehicle.h"

AppLogManager g_applog;

int main() {
  // Program entry point.
  g_applog.initialise();

  ARES_INFO("MAIN: Applog ready");
  ARES_INFO("MAIN: Program starts");
  /// We need to create instances of the main components of the robot as early
  /// as possible to guarantee the order of instantiation
  ARES_INFO("MAIN: Create Vehicle Board");
  BoardInterface& board = AresBoard::getInstance();
  ARES_INFO("MAIN: Create Vehicle using Board {}", board.getBoardName());
  Vehicle::instance(&board);

  ARES_INFO("MAIN: Instantiate Application");
  Application app;
  ARES_INFO("MAIN: Load Assets");
  auto image = sf::Image{};
  if (image.loadFromFile("assets/images/mouse-a.png")) {
    app.getWindow()->getRenderWindow()->setIcon(image.getSize().x, image.getSize().y, image.getPixelsPtr());
  }
  ARES_TRACE("size of Board = {}", sizeof(AresBoard));
  ARES_TRACE("size of Vehicle = {}", sizeof(Vehicle));
  ARES_TRACE("size of Mouse = {}", sizeof(Mouse));
  ARES_TRACE("size of MouseState = {}", sizeof(Mouse::MouseState));
  ARES_TRACE("size of Maze = {}", sizeof(Maze));
  //  printf("size of Board = %ul\n", sizeof(Mouse::instance()));
  ARES_INFO("MAIN: Run Application");
  app.run();
  ARES_INFO("MAIN: Shutdown ...");
  return 0;
}
