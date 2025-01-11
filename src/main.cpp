#include <imgui-SFML.h>
#include <SFML/Graphics.hpp>
#include <string>
#include "application/application.h"
// #include "application/robot-body.h"

/***
 * This app starts to take the 015 example and convert it to a more generic application structure.
 *
 * There are more generic classes used to assemble the application. In particular, main()
 * has little to do besides initialise the application and then run it. All the actual work
 * is done in the application class which manages its own window(s) and resources.
 *
 * The structure is derived from code associated with the book
 * "SFML Game Development by Example" by Maxime Lévesque
 *
 * The repository for the book code is at:
 * https://github.com/SFML/SFML-Game-Development-By-Example
 *
 */

//////////////////////////////////////////////////////////////////////////////////////////////////
///
///

//////////////////////////////////////////////////////////////////////////////////////////////////

int main() {
  // Program entry point.
  Application app;
  auto image = sf::Image{};
  if (image.loadFromFile("assets/images/mouse-a.png")) {
    app.getWindow()->getRenderWindow()->setIcon(image.getSize().x, image.getSize().y, image.getPixelsPtr());
  }

  app.run();
  return 0;
}
