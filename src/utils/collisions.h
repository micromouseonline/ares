//
// Created by peter on 18/11/24.
//

#ifndef COLLISIONS_H
#define COLLISIONS_H

#include <SFML/Graphics.hpp>

/**
 * In this file is a static struct that contains collision detection functions.  It is a struct
 * so that all the contents are public, and it is static in the sense that all the methods
 * are static and so can be called without instantiating the struct. By collecting the functions
 * in this way they all live in the same namespace and so should be called like:
 *
 *       sf::Vector2f point(50, 50);
 *       sf::RectangleShape rect(sf::Vector2f(100, 50));
 *       bool hit = Collisions::point_hits_rect(point, rect);
 *
 *       sf::FloatRect rect1(0, 0, 100, 100);
 *       sf::FloatRect rect2(50, 50, 100, 100);
 *       bool overlap = Collisions::rect_hits_rect(rect1, rect2);
 *
 *
 */

struct Collisions {
  /////////////////////////////////////////////////////////////////////
  ///
  /// For axis aligned rectangles:
  ///
  ///
  ///
  /// Although trivial, this wrapper function is included for consistency
  /// with other functions in this namespace.
  static bool point_hits_aligned_rect(sf::Vector2f& p, sf::RectangleShape& rect) { return rect.getGlobalBounds().contains(p); }

  /// Function to test if two axis-aligned rectangles overlap
  /// This is a test only valid for axis aligned rectangles
  /// SFML already can do this but this function is a wrapper for consistency
  /// other similar functions in this namespace.
  static bool aligned_rect_hits_aligned_rect(const sf::FloatRect& rect1, const sf::FloatRect& rect2) { return rect1.intersects(rect2); }

  /// This is a test only valid for axis aligned rectangles
  static bool circle_hits_aligned_rect(const sf::CircleShape& circle, const sf::RectangleShape& rect) {
    const sf::Vector2f circleCenter = circle.getPosition();

    const auto& rectBounds = rect.getGlobalBounds();
    const float rectLeft = rectBounds.left;
    const float rectTop = rectBounds.top;
    const float rectRight = rectBounds.left + rectBounds.width;
    const float rectBottom = rectTop + rectBounds.height;

    // Clamp the circle's center coordinates to the rectangle's edges
    const float closestX = std::clamp(circleCenter.x, rectLeft, rectRight);
    const float closestY = std::clamp(circleCenter.y, rectTop, rectBottom);

    // Calculate the squared distance from the circle's center to the closest point
    const float deltaX = circleCenter.x - closestX;
    const float deltaY = circleCenter.y - closestY;
    const float distanceSquared = deltaX * deltaX + deltaY * deltaY;

    // Compare squared distances to avoid costly sqrt operation
    const float radius = circle.getRadius();
    return distanceSquared < (radius * radius);
  }

  /////////////////////////////////////////////////////////////////////
  ///
  /// For arbitrarily rotated rectangles:
  ///
  /// This C++ function projects a 2D rectangle onto another 2D rectangle (axis).
  /// It is used by the separating axis theorem.
  /// The function returns a pair of floats that represent the minimum and maximum values of the projection.
  /// The first value is the minimum, and the second value is the maximum.
  /// The axis is a normalized vector that points in the direction of the projection.
  static std::pair<float, float> project_rect_onto_axis(const sf::RectangleShape& rect, const sf::Vector2f& axis) {
    std::vector<sf::Vector2f> points(4);
    for (size_t i = 0; i < 4; ++i) {
      points[i] = rect.getTransform().transformPoint(rect.getPoint(i));
    }
    float min = (points[0].x * axis.x + points[0].y * axis.y);
    float max = min;
    for (size_t i = 1; i < 4; ++i) {
      float projection = (points[i].x * axis.x + points[i].y * axis.y);
      if (projection < min)
        min = projection;
      if (projection > max)
        max = projection;
    }
    return std::make_pair(min, max);
  }

  /// This function is used by the separating axis theorem.
  /// It checks if there is a separating axis between two rectangles.
  /// The axis is a normalized vector that points in the direction of the projection.
  static bool is_separating_axis(const sf::RectangleShape& rect1, const sf::RectangleShape& rect2, const sf::Vector2f& axis) {
    auto projection1 = project_rect_onto_axis(rect1, axis);
    auto projection2 = project_rect_onto_axis(rect2, axis);

    return (projection1.second < projection2.first || projection2.second < projection1.first);
  }

  /// Function to check for overlap of rectangles regardless of their rotation
  /// This function is more general than the axis-aligned case at the top
  /// as it can handle rotated rectangles
  static bool rectangles_overlap(const sf::RectangleShape& rect1, const sf::RectangleShape& rect2) {
    std::vector<sf::Vector2f> axes = {rect1.getTransform().transformPoint(rect1.getPoint(1)) - rect1.getTransform().transformPoint(rect1.getPoint(0)),
                                      rect1.getTransform().transformPoint(rect1.getPoint(3)) - rect1.getTransform().transformPoint(rect1.getPoint(0)),
                                      rect2.getTransform().transformPoint(rect2.getPoint(1)) - rect2.getTransform().transformPoint(rect2.getPoint(0)),
                                      rect2.getTransform().transformPoint(rect2.getPoint(3)) - rect2.getTransform().transformPoint(rect2.getPoint(0))};

    for (auto& axis : axes) {
      if (is_separating_axis(rect1, rect2, sf::Vector2f(-axis.y, axis.x))) {
        return false;  // There is a separating axis, so they do not overlap
      }
    }
    return true;  // No separating axis found, so they overlap
  }

  /////////////////////////////////////////////////////////////////////

  /// This C++ function projects a 2D vector onto another 2D vector (axis).
  /// It returns the scalar value of the projection, which represents the
  /// length of the vector's shadow on the axis.
  ///
  /// The function normalizes the axis vector to ensure a proper projection.
  /// If the axis vector is zero, the function returns infinity.
  /// In simpler terms, it calculates how much of the vector is "pointing" in
  /// the direction of the axis.
  static float project_vector(const sf::Vector2f& vector, const sf::Vector2f& axis) {
    // Normalize the axis to ensure a proper projection
    float axisLengthSquared = axis.x * axis.x + axis.y * axis.y;
    if (std::fabs(axisLengthSquared) <= std::numeric_limits<float>::epsilon()) {
      std::cerr << "Axis vector cannot be zero in project_vector.\n";
      return std::numeric_limits<float>::infinity();
    }
    return (vector.x * axis.x + vector.y * axis.y) / std::sqrt(axisLengthSquared);
  }

  /// This function returns the 4 vertices of a rectangle in world space,
  /// taking into account the rectangle's position, rotation, and scale.
  static std::vector<sf::Vector2f> get_rectangle_vertices(const sf::RectangleShape& rect) {
    std::vector<sf::Vector2f> vertices(4);
    for (size_t i = 0; i < 4; ++i) {
      vertices[i] = rect.getTransform().transformPoint(rect.getPoint(i));
    }
    return vertices;
  }

  /// This more generalised case is computationally more expensive than the axis-aligned case at the top
  /// as it has to handle rotated rectangles. It uses the Separating Axis Theorem (SAT)
  /// https://programmerart.weebly.com/separating-axis-theorem.html
  /// Although it looks complex, it is moderately easy to understand. This is not an inexpensive
  /// operation so, in general, look to find a smaller set of objects to compare
  /// rather than iterating through everything.
  static bool circle_hits_rotated_rect(const sf::CircleShape& circle, const sf::RectangleShape& rect) {
    // first test the bounding box in case we are not even close
    if (!circle_hits_aligned_rect(circle, rect)) {
      return false;
    }
    // Circle properties
    sf::Vector2f circleCenter = circle.getPosition();
    float circleRadius = circle.getRadius();

    // Get rectangle vertices and edges
    auto vertices = get_rectangle_vertices(rect);
    std::vector<sf::Vector2f> edges = {vertices[1] - vertices[0], vertices[2] - vertices[1], vertices[3] - vertices[2], vertices[0] - vertices[3]};

    // Add the circle's center-to-vertex axes for SAT
    for (const auto& vertex : vertices) {
      edges.push_back(circleCenter - vertex);
    }

    // Check for overlap on all axes
    for (const auto& edge : edges) {
      sf::Vector2f axis(-edge.y, edge.x);  // Perpendicular axis

      // Project circle onto the axis
      float circleProjection = project_vector(circleCenter, axis);
      float circleMin = circleProjection - circleRadius;
      float circleMax = circleProjection + circleRadius;

      // Project rectangle onto the axis
      float rectMin = std::numeric_limits<float>::max();
      float rectMax = std::numeric_limits<float>::lowest();
      for (const auto& vertex : vertices) {
        float projection = project_vector(vertex, axis);
        rectMin = std::min(rectMin, projection);
        rectMax = std::max(rectMax, projection);
      }

      // Check for overlap
      if (circleMax < rectMin || circleMin > rectMax) {
        return false;  // No overlap on this axis, so no collision
      }
    }

    return true;  // Overlap on all axes, collision detected
  }
  /////////////////////////////////////////////////////////////////////

};  // struct Collisions

#endif  // COLLISIONS_H
