//
// Created by peter on 22/11/24.
//

#ifndef EVENT_OBSERVER_H
#define EVENT_OBSERVER_H

#include <SFML/Graphics.hpp>

/***
 * The Event Observer is a way for the window to notify observers when an event occurs.
 * Primarily these are SFML Events like button presses or mouse movement. Other,
 * user defined events can also be added. In the Robot simulation for instance, a
 * crash might notify the main thread to stop the simulation.
 */
enum EventType {
  SFML_EVENT,   // Any of the built-in SFML events
  USER_EVENT,   // a place holder as a reminder other types are possible
  ROBOT_EVENT,  //
};

/***
 * Events will have data associated with them. The event type is needed so that
 * the handler knows what else to look for since not all events carry the same data.
 *
 * For example: A mouse event might carry the position of the mouse.
 */
struct Event {
  EventType type;
  sf::Event event;
  // And other details as needed.
  int user_value = 0;
};

/***
 * The Event Observer interface is very simple. Any class that wished to handle an
 * event notification must inherit from this class and implement the OnEvent method.
 * That is all they have to do. We are not concerned about passing events from one
 * observer to another. If you sign up to be notified of an event, you will get it
 * and are assumed to have dealt with it.
 * It would be possible to extend the system so that events of different types could
 * be subscribed to by different classes.
 */
class IEventObserver {
 public:
  virtual ~IEventObserver() = default;
  virtual void OnEvent(const Event& event) = 0;
};

#endif  // EVENT_OBSERVER_H
