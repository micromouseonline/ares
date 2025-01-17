//
// Created by peter on 18/11/24.
//

#ifndef COLLISIONS_H
#define COLLISIONS_H

#include "SFML/Graphics.hpp"

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
  static bool pointHitsAlignedRect(sf::Vector2f& p, sf::RectangleShape& rect) {
    return rect.getGlobalBounds().contains(p);  //
  }

  /// Function to test if two axis-aligned rectangles overlap
  /// This is a test only valid for axis aligned rectangles
  /// SFML already can do this but this function is a wrapper for consistency
  /// other similar functions in this namespace.
  static bool alignedRectHitsAlignedRect(const sf::FloatRect& rect1, const sf::FloatRect& rect2) {
    return rect1.intersects(rect2);  //
  }

  /// This is a test only valid for axis aligned rectangles
  static bool circleHitsAlignedRect(const sf::CircleShape& circle, const sf::RectangleShape& rect) {
    const sf::Vector2f circle_center = circle.getPosition();

    const auto& rect_bounds = rect.getGlobalBounds();
    const float rect_left = rect_bounds.left;
    const float rect_top = rect_bounds.top;
    const float rect_right = rect_bounds.left + rect_bounds.width;
    const float rect_bottom = rect_top + rect_bounds.height;

    // Clamp the circle's center coordinates to the rectangle's edges
    const float closest_x = std::clamp(circle_center.x, rect_left, rect_right);
    const float closest_y = std::clamp(circle_center.y, rect_top, rect_bottom);

    // Calculate the squared distance from the circle's center to the closest point
    const float delta_x = circle_center.x - closest_x;
    const float delta_y = circle_center.y - closest_y;
    const float distance_squared = delta_x * delta_x + delta_y * delta_y;

    // Compare squared distances to avoid costly sqrt operation
    const float radius = circle.getRadius();
    return distance_squared < (radius * radius);
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
  static std::pair<float, float> projectRectOntoAxis(const sf::RectangleShape& rect, const sf::Vector2f& axis) {
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
  static bool isSeparatingAxis(const sf::RectangleShape& rect1, const sf::RectangleShape& rect2, const sf::Vector2f& axis) {
    auto projection1 = projectRectOntoAxis(rect1, axis);
    auto projection2 = projectRectOntoAxis(rect2, axis);

    return (projection1.second < projection2.first || projection2.second < projection1.first);
  }

  /// Function to check for overlap of rectangles regardless of their rotation
  /// This function is more general than the axis-aligned case at the top
  /// as it can handle rotated rectangles
  static bool rectanglesOverlap(const sf::RectangleShape& rect1, const sf::RectangleShape& rect2) {
    std::vector<sf::Vector2f> axes = {rect1.getTransform().transformPoint(rect1.getPoint(1)) - rect1.getTransform().transformPoint(rect1.getPoint(0)),
                                      rect1.getTransform().transformPoint(rect1.getPoint(3)) - rect1.getTransform().transformPoint(rect1.getPoint(0)),
                                      rect2.getTransform().transformPoint(rect2.getPoint(1)) - rect2.getTransform().transformPoint(rect2.getPoint(0)),
                                      rect2.getTransform().transformPoint(rect2.getPoint(3)) - rect2.getTransform().transformPoint(rect2.getPoint(0))};

    for (auto& axis : axes) {
      if (isSeparatingAxis(rect1, rect2, sf::Vector2f(-axis.y, axis.x))) {
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
  static float projectVectorOntoAxis(const sf::Vector2f& vector, const sf::Vector2f& axis) {
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
  static std::vector<sf::Vector2f> getRectangleVertices(const sf::RectangleShape& rect) {
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
  static bool circleHitsGenericRect(const sf::CircleShape& circle, const sf::RectangleShape& rect) {
    // first test the bounding box in case we are not even close
    if (!circleHitsAlignedRect(circle, rect)) {
      return false;
    }
    // Circle properties
    sf::Vector2f circleCenter = circle.getPosition();
    float circleRadius = circle.getRadius();

    // Get rectangle vertices and edges
    auto vertices = getRectangleVertices(rect);
    std::vector<sf::Vector2f> edges = {vertices[1] - vertices[0], vertices[2] - vertices[1], vertices[3] - vertices[2], vertices[0] - vertices[3]};

    // Add the circle's center-to-vertex axes for SAT
    for (const auto& vertex : vertices) {
      edges.push_back(circleCenter - vertex);
    }

    // Check for overlap on all axes
    for (const auto& edge : edges) {
      sf::Vector2f axis(-edge.y, edge.x);  // Perpendicular axis

      // Project circle onto the axis
      float circleProjection = projectVectorOntoAxis(circleCenter, axis);
      float circleMin = circleProjection - circleRadius;
      float circleMax = circleProjection + circleRadius;

      // Project rectangle onto the axis
      float rectMin = std::numeric_limits<float>::max();
      float rectMax = std::numeric_limits<float>::lowest();
      for (const auto& vertex : vertices) {
        float projection = projectVectorOntoAxis(vertex, axis);
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

  /***
   * This is the meat of the sensor simulation. A single ray is tested for intersection
   * with an axis-aligned rectangle. Two such tests are needed, one for the far
   * side and one for the near side. We only care about the closest one and we
   * do not know if it is the first one found or the second so we find both
   * and work it out after.
   * TODO: I do not fully understand this function.
   * @param rectangle
   * @param ray_start - the location of the start of the ray
   * @param ray_dir - as a normalised vector
   * @return the distance to the closest intersection or the max range if
   * there is no intersection
   */
  static float getRayDistanceToAlignedRectangle(const sf::Vector2f& ray_start, const sf::Vector2f& ray_dir, const sf::FloatRect& rectangle, float max_range) {
    sf::FloatRect bounds = rectangle;
    sf::Vector2f rectMin(bounds.left, bounds.top);
    sf::Vector2f rectMax(bounds.left + bounds.width, bounds.top + bounds.height);

    float tmin = -std::numeric_limits<float>::infinity();
    float tmax = std::numeric_limits<float>::infinity();

    // Handle the x axis (left-right)
    if (std::abs(ray_dir.x) > 1e-6) {  // Ray is not parallel to the y-axis
      float t1 = (rectMin.x - ray_start.x) / ray_dir.x;
      float t2 = (rectMax.x - ray_start.x) / ray_dir.x;

      if (t1 > t2)
        std::swap(t1, t2);
      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);

      if (tmin > tmax) {
        // No intersection on the x axis
        return max_range;
      }
    } else {  // Ray is parallel to the x-axis
      if (ray_start.x < rectMin.x || ray_start.x > rectMax.x) {
        // No intersection if the ray is outside the x bounds
        return max_range;
      }
    }

    // Handle the y axis (top-bottom)
    if (std::abs(ray_dir.y) > 1e-6) {  // Ray is not parallel to the y-axis
      float t1 = (rectMin.y - ray_start.y) / ray_dir.y;
      float t2 = (rectMax.y - ray_start.y) / ray_dir.y;

      if (t1 > t2)
        std::swap(t1, t2);
      tmin = std::max(tmin, t1);
      tmax = std::min(tmax, t2);

      if (tmin > tmax) {
        // No intersection on the y axis
        return max_range;
      }
    } else {  // Ray is parallel to the y-axis
      if (ray_start.y > rectMin.y || ray_start.y < rectMax.y) {
        // No intersection if the ray is outside the y bounds
        return max_range;
      }
    }

    // If the intersection is behind the ray origin, return max_range
    if (tmax < 0) {
      return max_range;
    }

    // Return the nearest intersection (either tmin or tmax), ensuring it's in front of the ray
    return std::min(tmin >= 0 ? tmin : tmax, max_range);
  }

};  // struct Collisions

#endif  // COLLISIONS_H
