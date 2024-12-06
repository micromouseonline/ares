//
// Created by peter on 22/11/24.
//

#ifndef WINDOW_H
#define WINDOW_H
#include <SFML/Graphics.hpp>
#include <string>
#include "event_observer.h"

/***
 * Implements a Window class that wraps sf::RenderWindow to provide a clean interface
 * and features including window creation and destruction, window updates, window drawing
 * and window closing.
 *
 * This class can also be observed by instances of an IEventObserver. A list of observers
 * is maintained and any class can register as an observer by calling the AddObserver method
 * giving it a pointer to an instance of an IEventObserver.
 *
 * All observers will be notified of any events that occur in the window by calling
 * their OnEvent() method. The event will be passed as a parameter. Events that are handled
 * by window will not be notified. this is an option and all events could be notified if
 * necessary. Don't make it complicated though.
 */
class Window {
 public:
  Window();
  Window(const std::string& title, const sf::Vector2u& size);
  ~Window();

  void SetTitle(const std::string& title);

  /// Clears the window background
  void beginDraw();

  /// Displays the changes made to the window
  void endDraw();

  /// This seems to suck up the events. needs more work?
  /// Process any events
  void update();

  /// The normal close events merely set a flag that can be tested with this getter.
  /// That allows the application to do some cleanup before exiting.
  bool isDone() const;

  sf::Vector2u getWindowSize();

  sf::RenderWindow* getRenderWindow() {
    return &m_window;  //
  }

  /// Used to draw any drawable object into the window. The actual window variable
  /// is private so we do not have direct access to it
  void draw(sf::Drawable& l_drawable);

  sf::View& getMazeView() {
    return m_maze_view;  //
  }

  sf::View& getUIView() {
    return m_default_view;  //
  }

  /// Any class that inherits from IEventObserver can register as an observer
  void addObserver(IEventObserver* observer);

 private:
  void setup(const std::string title, const sf::Vector2u& size);

  void create();

  void destroy();

  void updateViews();

  void notifyObservers(const AppEvent& event);

  sf::RenderWindow m_window;
  sf::Vector2u m_window_size;
  std::string m_window_title;
  sf::View m_maze_view;
  sf::View m_ui_view;
  sf::View m_default_view;
  bool m_is_done;
  /// Note that there will be trouble if an observer registers and then gets destroyed
  /// because the vector will still have a pointer to it. The solution is to use a shared_ptr
  std::vector<IEventObserver*> m_observers;
};

#endif  // IMGUI_SFML_STARTER_WINDOW_H
