//
// Created by peter on 22/11/24.
//

#include "window.h"
#include "application/configuration.h"

/***
 * The window manager really just looks after things like creating and destroying
 * the main window.
 * It also handles events and updating the window.
 */
Window::Window() {
  m_window.create(conf::VideoMode, "Window from constructor");
}

Window::Window(const std::string& title, const sf::Vector2u& size) {
  setup(title, size);
}

Window::~Window() {
  destroy();
}

void Window::setup(const std::string title, const sf::Vector2u& size) {
  m_window_title = title;
  m_window_size = size;
  m_is_done = false;
  /// To change the properties, you have to re-create the screen
  destroy();
  create();
  updateViews();
  m_window.setFramerateLimit(conf::FrameRate);
}

void Window::create() {
  sf::ContextSettings settings;
  settings.antialiasingLevel = 8;  // the number of multi-samplings to use. 4 is probably fine
  int style = sf::Style::Titlebar + sf::Style::Close + sf::Style::Resize;
  m_window.create({m_window_size.x, m_window_size.y, 32}, m_window_title, style, settings);
}

void Window::destroy() {
  m_window.close();
}

void Window::updateViews() {
  m_maze_view = conf::MazeView;
  float width = (float)m_window.getSize().x;
  float height = (float)m_window.getSize().y;
  float scale = 1.0f;
  sf::FloatRect vp_rect;
  vp_rect.left = conf::WindowPadding / width;
  vp_rect.top = conf::WindowPadding / height;
  vp_rect.width = scale * conf::MazeViewScreenSize / width;
  vp_rect.height = scale * conf::MazeViewScreenSize / height;
  m_maze_view.setViewport(vp_rect);
  float visible_width = (float)m_window.getSize().x;
  float visible_height = (float)m_window.getSize().y;
  m_default_view = sf::View(sf::FloatRect(0, 0, visible_width, visible_height));
}

void Window::beginDraw() {
  m_window.clear(conf::WindowBackGround);
}
void Window::endDraw() {
  m_window.display();
}

bool Window::isDone() const {
  return m_is_done;
}

void Window::draw(sf::Drawable& l_drawable) {
  m_window.draw(l_drawable);
}

sf::Vector2u Window::getWindowSize() {
  return m_window_size;
}

void Window::update() {
  sf::Event event;
  while (m_window.pollEvent(event)) {
    if (event.type == sf::Event::Closed) {
      m_is_done = true;
    } else if (event.type == sf::Event::Resized) {
      updateViews();
    } else {
      Event e = {EventType::SFML_EVENT, event, {}};
      notifyObservers(e);
    }
  }
}

void Window::SetTitle(const std::string& title) {
  m_window_title = title;
  m_window.setTitle(m_window_title);
}

void Window::addObserver(IEventObserver* observer) {
  m_observers.push_back(observer);
}

void Window::notifyObservers(const Event& event) {
  for (auto& observer : m_observers) {
    observer->onEvent(event);
  }
}
