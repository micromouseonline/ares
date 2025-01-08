#include <cassert>
#include <iostream>
#include <memory>
#include <vector>

/***
 * An example of how to use a factory and a manager to create instances of
 * classes derived from some pure virtual base class.
 *
 * The base class here is a Shape and there are a number of specific classes
 * derived from that. Each has an update method that returns a value.
 *
 * The manager maintains a queue of Shapes. As Shape is the base class
 * the queue can hold any derived class as well.
 *
 * A queue is used to give the items an order so that they can be processed
 * in the order they arrive.
 *
 * A method is also provided to allow all the items to be processed in a batch.
 *
 * You can get a reference to the item at the head of the queue and run whatever
 * methods it provides. When you are doene with that item, it can be popped off
 * the queue.
 *
 * Alternatively, the item at the head can be popped of into a suitable pointer
 * and handled that way.
 *
 * The items in the queue are stored on the heap and the queue is just a
 * vector of unique_ptr to each item. The use of std::unique_ptr ensures that
 * the items in the queue are correctly destroyed when they go out of scope
 * and there will be no memory leaks.
 *
 * Normally the manager would be given a size for the number of items the queue
 * can hold when it is constructed. the queue size can be changed later but this
 * is going to take time and will cause heap fragmentation so try and avoid it.
 *
 * The concrete shape instances are created by a factory class. You give it the
 * enum for a particular type of shape and it will create that instance
 * and return a unique poiter to it. That pointer can be used to initialise the
 * object if desired and then it is given to the manager for processinf
 *
 * Each shape has a number of lives. A life is lost after each call to its
 * update() method. You initialise each shape with a number of lives by
 * calling he init() method before handing the shape to the manager.
 *
 * You can go through the manager queue and work out how many ticks in total
 * will elapse before the entire queue is consumed.
 */

// Base class
class Shape {
 public:
  Shape() : m_lifespan(0) {};

  virtual ~Shape() = default;  // Virtual destructor

  virtual void init(int lives) = 0;

  virtual void update() = 0;  // Pure virtual function

  virtual std::string name() = 0;

  int duration() {
    reset();
    int ticks = 0;
    while (!isFinished()) {
      update();
      ticks++;
    }
    reset();
    return ticks;
  }

  void reset() {
    m_ticks = 0;
  }

  bool isFinished() {
    return m_ticks >= m_lifespan;
  }

 protected:
  int m_lifespan;
  int m_ticks;
};

// Derived classes
//----------------------------------------------------------
class Hexagon : public Shape {
 public:
  void init(int lives) override {
    m_lifespan = lives;
  }

  std::string name() override {
    return "Hexagon";
  }

  void update() override {
    std::cout << m_ticks << ", ";
    m_ticks++;
  }
};

//----------------------------------------------------------
class Triangle : public Shape {
 public:
  void init(int lives) override {
    m_lifespan = lives;
  }
  std::string name() override {
    return "Triangle";
  }

  void update() override {
    std::cout << m_ticks << ", ";
    m_ticks++;
  }
};

//----------------------------------------------------------
class Rectangle : public Shape {
 public:
  void init(int lives) override {
    m_lifespan = lives;
  }
  std::string name() override {
    return "Rectangle";
  }

  void update() override {
    std::cout << m_ticks << ", ";
    m_ticks++;
  }
};

//----------------------------------------------------------
// Enum for different types of shapes
enum class ShapeType { Hexagon, Triangle, Rectangle, SHAPE_COUNT };

//----------------------------------------------------------
// Factory class
class ShapeFactory {
 public:
  std::unique_ptr<Shape> create(ShapeType type) {
    switch (type) {
      case ShapeType::Hexagon:
        return std::make_unique<Hexagon>();
      case ShapeType::Triangle:
        return std::make_unique<Triangle>();
      case ShapeType::Rectangle:
        return std::make_unique<Rectangle>();
      default:
        return nullptr;
    }
  }
};

//////////////////////////////////////////////////////////////////////////

class ShapeManager {
 public:
  ShapeManager(int size) : m_shapes(size), m_capacity(size), m_head(0), m_tail(0), m_size(0) {
  }

  void clear() {
    while (m_size > 0) {
      pop();
    }
  }

  bool push_back(std::unique_ptr<Shape> shape) {
    if (isFull()) {
      return false;
    }
    m_shapes[m_tail] = std::move(shape);
    m_tail = (m_tail + 1) % m_capacity;
    m_size++;
    return true;
  }

  Shape& peek() {
    return *(m_shapes[m_head]);
  }

  std::unique_ptr<Shape> pop() {
    if (isEmpty()) {
      return nullptr;
    }
    std::unique_ptr shape = std::move(m_shapes[m_head]);
    m_head = (m_head + 1) % m_capacity;
    m_size--;
    return shape;
  }

  int update() {
    //    if (isEmpty()) {
    //      return 0;
    //    }
    if (peek().isFinished()) {
      std::cout << "\n";
      pop();
    }
    if (!isEmpty()) {
      peek().update();
    }
    return 0;
  }

  int getDuration() {
    if (isEmpty()) {
      return 0;
    }
    int result = 0;
    int i = m_head;
    do {
      result += m_shapes[i]->duration();
      i = (i + 1) % m_capacity;
    } while (i != m_tail);
    return result;
  }

  bool isEmpty() {
    return (m_size == 0);
  }

  bool isFull() {
    return m_size >= m_capacity;
  }

  int size() {
    return m_size;
  }

  int capacity() {
    return m_capacity;
  }

 private:
  std::vector<std::unique_ptr<Shape>> m_shapes;
  int m_capacity;
  int m_head;
  int m_tail;
  int m_size;
};

//////////////////////////////////////////////////////////////////////////

ShapeType randomShape() {
  int i = std::rand() % (int)ShapeType::SHAPE_COUNT;
  return static_cast<ShapeType>(i);
}

int main() {
  ShapeFactory factory;
  ShapeManager manager(8);

  /// create a shape, initialise it, give it to the manager
  std::unique_ptr<Shape> new_shape;

  new_shape = factory.create(ShapeType::Hexagon);
  new_shape->init(6);
  manager.push_back(std::move(new_shape));

  new_shape = factory.create(ShapeType::Triangle);
  new_shape->init(3);
  manager.push_back(std::move(new_shape));

  new_shape = factory.create(ShapeType::Rectangle);
  new_shape->init(4);
  manager.push_back(std::move(new_shape));

  std::cout << "\nTotal duration ... \n";
  int result = manager.getDuration();
  std::cout << "\n  ... is " << result << "\n\n";

  std::cout << "\nprocessing the queue ... \n";
  while (!manager.isEmpty()) {
    manager.update();
  }
  std::cout << "   ... finished\n\n";

  return 0;
}
