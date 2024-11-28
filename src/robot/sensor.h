#ifndef SENSOR_H
#define SENSOR_H

#include <SFML/Graphics.hpp>
#include <vector>
#include "utils.h"

const float DEG_TO_RAD = 3.14159265359f / 180.0f;

class Sensor {
 public:
  Sensor(sf::Vector2f origin, float angle, float half_angle = 5.0f, int ray_count = 16)
      : m_origin(origin), m_angle(angle), m_half_angle(half_angle), m_rays(ray_count + 1) {
    m_vertices.resize(m_rays);
    m_vertices.setPrimitiveType(sf::TriangleFan);
    m_vertices[0].position = m_origin;  // First vertex is the origin
    m_vertices[0].color = sf::Color(128, 0, 128);

    m_max_range = 500.0f;
    m_power = 0.0f;
    m_distance = 0.0f;
  }

  void set_origin(const sf::Vector2f& origin) {
    m_origin = origin;
    m_vertices[0].position = m_origin;  // Update origin vertex
  }

  /// The sensor fan will be centered on this angle and spread +/- the half-angle
  void set_angle(float angle) { m_angle = angle; }

  void set_half_angle(float half_angle) { m_half_angle = half_angle; }

  void set_ray_count(int ray_count) { m_rays = ray_count; }

  [[nodiscard]] float power() const { return m_power; }
  [[nodiscard]] float distance() const { return m_distance; }

  /***
   * generate a complete sensor fan for every rectangle in the supplied
   * vector. This is surprisingly fast. Even so, in the actual simulation
   * the rectangles passed in will only be those surrounding cells.
   * The eight immediate neighbours should be enough so a maximum of
   * 32 rectangles is required.
   * @param obstacles
   */
  void update(const std::vector<sf::RectangleShape>& obstacles) {
    // Calculate angular increment for rays
    float startAngle = (m_angle - m_half_angle) * DEG_TO_RAD;
    float endAngle = (m_angle + m_half_angle) * DEG_TO_RAD;
    float angleIncrement = (endAngle - startAngle) / float(m_rays - 1);

    float total_power = 0;
    float total_distance = 0;

    for (int i = 1; i < m_rays; ++i) {  // Remember to skip origin (index 0)
      float angle = startAngle + float(i - 1) * angleIncrement;
      sf::Vector2f dir = {std::cos(angle), std::sin(angle)};

      float closestHit = m_max_range;
      for (const auto& rect : obstacles) {
        float distance = test_to_rect(rect, dir);
        if (distance < closestHit) {
          closestHit = distance;
        }
      }
      // Update the ray endpoint
      sf::Vector2f hitPosition = m_origin + dir * closestHit;
      m_vertices[i].position = hitPosition;
      m_vertices[i].color = sf::Color(128, 0, 128, 255 * (1.0f - closestHit / m_max_range));

      /// Accumulate distance and power for averaging
      /// you cannot do te power afterwards form the average distance
      /// because power is not linear with distance.
      /// perhaps we should only record power and always calculate
      /// distance - as in the real mouse
      /// TODO: implement a proper function that calculates power
      float power = 855.0f / closestHit;
      power = power * power;
      total_power += power;
      total_distance += closestHit;
    }

    m_distance = std::min(total_distance / float(m_rays - 1), m_max_range);
    m_power = std::min(total_power / float(m_rays - 1), 1024.0f);
  }

  /// The sensor will be drawn as a triangle fan. This is really
  /// fast because the GPU does all the work from the vertex list
  void draw(sf::RenderTarget& renderTarget) const { renderTarget.draw(m_vertices); }

 private:
  /***
   * This is the meat of the business. A single ray is tested for intersection
   * with an axis-aligned rectangle. Two such tests are needed, one for the far
   * side and one for the near side. We only care about the closest one and we
   * do not know if it is the first one found or the second so we find both
   * and work it out after.
   * TODO: I do not fully understand this function.
   * @param rectangle
   * @param ray_dir - as a normalised vector
   * @return the distance to the closest intersection or the maze range if
   * there is no intersection
   */
  float test_to_rect(const sf::RectangleShape& rectangle, const sf::Vector2f& ray_dir) {
    sf::FloatRect bounds = rectangle.getGlobalBounds();
    sf::Vector2f rectMin(bounds.left, bounds.top);
    sf::Vector2f rectMax(bounds.left + bounds.width, bounds.top + bounds.height);

    float tmin = -std::numeric_limits<float>::infinity();
    float tmax = std::numeric_limits<float>::infinity();

    for (int i = 0; i < 2; ++i) {
      float origin = (i == 0) ? m_origin.x : m_origin.y;
      float dir = (i == 0) ? ray_dir.x : ray_dir.y;
      float min = (i == 0) ? rectMin.x : rectMin.y;
      float max = (i == 0) ? rectMax.x : rectMax.y;

      if (std::abs(dir) < 1e-6) {
        if (origin < min || origin > max) {
          // No intersection
          return m_max_range;
        }
      } else {
        float t1 = (min - origin) / dir;
        float t2 = (max - origin) / dir;

        if (t1 > t2)
          std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);

        if (tmin > tmax) {
          // No intersection
          return m_max_range;
        }
      }
    }
    if (tmax < 0) {
      // Intersection behind the ray origin. Are we INSIDE the box?
      return m_max_range;
    }
    return std::min(tmin >= 0 ? tmin : tmax, m_max_range);  // Intersection behind the ray origin
  }

  sf::Vector2f m_origin = {0, 0};
  float m_angle;
  float m_half_angle;
  float m_max_range;
  int m_rays;
  float m_power;
  float m_distance;

  sf::VertexArray m_vertices;
};

#endif  // SENSOR_H
