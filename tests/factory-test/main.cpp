#include <iostream>
#include <memory>
#include <vector>

// Base class
class Shape {
 public:
  virtual ~Shape() = default;       // Virtual destructor
  virtual void update() const = 0;  // Pure virtual function
};

// Derived classes
class Circle : public Shape {
 public:
  void update() const override {
    std::cout << "Circle is updating!" << std::endl;
  }
};

class Triangle : public Shape {
 public:
  void update() const override {
    std::cout << "Triangle is updating!" << std::endl;
  }
};

class Rectangle : public Shape {
 public:
  void update() const override {
    std::cout << "Rectangle is updating!" << std::endl;
  }
};

// Enum for different types of shapes
enum class ShapeType { Circle, Triangle, Rectangle };

// Factory class
class ShapeFactory {
 public:
  std::unique_ptr<Shape> create(ShapeType type) {
    switch (type) {
      case ShapeType::Circle:
        return std::make_unique<Circle>();
      case ShapeType::Triangle:
        return std::make_unique<Triangle>();
      case ShapeType::Rectangle:
        return std::make_unique<Rectangle>();
      default:
        return nullptr;
    }
  }
};

// Class managing the vector and position tracker
class ShapeManager {
 public:
  ShapeManager() : currentIndex(0) {
  }

  void addShape(std::unique_ptr<Shape> shape) {
    shapeVector.push_back(std::move(shape));
  }

  Shape* getNextShape() {
    if (currentIndex < shapeVector.size()) {
      return shapeVector[currentIndex++].get();
    }
    return nullptr;
  }

 private:
  std::vector<std::unique_ptr<Shape>> shapeVector;
  size_t currentIndex;
};

// Function to process and update shapes using getNextShape
void processShapes(ShapeManager& manager) {
  Shape* shape = nullptr;
  while ((shape = manager.getNextShape()) != nullptr) {
    shape->update();
  }
}

int main() {
  ShapeFactory factory;
  ShapeManager manager;

  // Create and add shapes to the manager
  manager.addShape(factory.create(ShapeType::Circle));
  manager.addShape(factory.create(ShapeType::Triangle));
  manager.addShape(factory.create(ShapeType::Rectangle));

  // Process and update all shapes in the manager
  processShapes(manager);

  return 0;
}
