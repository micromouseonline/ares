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
  void BeginDraw();

  /// Displays the changes made to the window
  void EndDraw();

  /// This seems to suck up the events. needs more work?
  /// Process any events
  void Update();

  /// The normal close events merely set a flag that can be tested with this getter.
  /// That allows the application to do some cleanup before exiting.
  bool IsDone() const;

  void ToggleFullscreen();
  bool IsFullscreen() const;

  sf::Vector2u GetWindowSize();
  sf::RenderWindow* GetRenderWindow() { return &m_window; }

  /// Used to draw any drawable object into the window. The actual window variable
  /// is private so we do not have direct access to it
  void Draw(sf::Drawable& l_drawable);

  /// Any class that inherits from IEventObserver can register as an observer
  void AddObserver(IEventObserver* observer);

 private:
  void Setup(const std::string title, const sf::Vector2u& size);
  void Create();
  void Destroy();
  void NotifyObservers(const Event& event);

  sf::RenderWindow m_window;
  sf::Vector2u m_windowSize;
  std::string m_windowTitle;
  bool m_isDone;
  /// Note that there will be trouble if an observer registers and then gets destroyed
  /// because the vector will still have a pointer to it. The solution is to use a shared_ptr
  std::vector<IEventObserver*> m_observers;
};

#endif  // IMGUI_SFML_STARTER_WINDOW_H
