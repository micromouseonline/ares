//
// Created by peter on 22/11/24.
//

#include "window.h"

/***
 * The window manager really just looks after things like creating and destroying
 * the main window.
 * It also handles events and updating the window.
 */
Window::Window() {
  m_window.create(sf::VideoMode(640, 480), "Window from constructor");
}

Window::Window(const std::string& title, const sf::Vector2u& size) {
  Setup(title, size);
}

Window::~Window() {
  Destroy();
}

void Window::Setup(const std::string title, const sf::Vector2u& size) {
  m_windowTitle = title;
  m_windowSize = size;
  m_isDone = false;
  /// To change the properties, you have to re-create the screen
  Destroy();
  Create();
  sf::View view = m_window.getDefaultView();
  sf::Vector2f originalSize = view.getSize();                  // Scale the view to 25%
  view.setSize(originalSize.x * 3.0f, originalSize.y * 3.0f);  // Center the view (optional, depending on desired effect)
  sf::Vector2f newSize = view.getSize();                       // Scale the view to 25%

  view.setCenter(newSize.x * 0.5f, newSize.y * 0.5f);  // Apply the view to the window
  m_window.setView(view);
  m_window.setFramerateLimit(60);
}

void Window::Create() {
  sf::ContextSettings settings;
  settings.antialiasingLevel = 8;  // the number of multi-samplings to use. 4 is probably fine
  int style = sf::Style::Titlebar + sf::Style::Close;
  m_window.create({m_windowSize.x, m_windowSize.y, 32}, m_windowTitle, style, settings);
}

void Window::Destroy() {
  m_window.close();
}

void Window::BeginDraw() {
  m_window.clear(sf::Color::Black);
}
void Window::EndDraw() {
  m_window.display();
}

bool Window::IsDone() const {
  return m_isDone;
}

void Window::Draw(sf::Drawable& l_drawable) {
  m_window.draw(l_drawable);
}

sf::Vector2u Window::GetWindowSize() {
  return m_windowSize;
}

void Window::ToggleFullscreen() {
  /// This is disabled for now because it can leave your screen
  /// in the same resolution as the window size which is no fun.
  //  m_isFullscreen = !m_isFullscreen;
  //  Destroy();
  //  Create();
}

void Window::Update() {
  sf::Event event;
  while (m_window.pollEvent(event)) {
    if (event.type == sf::Event::Closed) {
      m_isDone = true;
    } else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::F5) {
      ToggleFullscreen();
    } else {
      Event e = {EventType::SFML_EVENT, event, {}};
      NotifyObservers(e);
    }
  }
}

void Window::SetTitle(const std::string& title) {
  m_windowTitle = title;
  m_window.setTitle(m_windowTitle);
}

void Window::AddObserver(IEventObserver* observer) {
  m_observers.push_back(observer);
}

void Window::NotifyObservers(const Event& event) {
  for (auto& observer : m_observers) {
    observer->OnEvent(event);
  }
}
