#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include "SFML/Graphics.hpp"
#include "common/utils.h"
#include "configuration.h"
/**
 * The RobotWallSensor is a mocked device that calculates sensor data for the simulated
 * robot. Instances of the lass represent a single sensor that can calculate the
 * reflected light power from nearby walls. It does this by finding the distance to the
 * nearest obstacle for a fan of rays emitted from the sensor location. The number of rays
 * and their angular spread can be specified. The actual response is the average of the
 * distances for each ray.
 *
 * The power is calculated from an inverse square model for the relationship between
 * reflected power and distance.
 *
 * An individual sensor must also be given a position and an orientation. These are
 * determined by the position and orientation of the Robot to which they belong.
 *
 * The Robot has a list of sensors positions and, for each, a set of offsets -
 * x,y,theta - that are relative to the Robot's centre of rotation and current heading.
 *
 * The Application can query those parameters. The CalculateSensors callback will
 * query the robot for the location of each sensor and then perform the calculation.
 *
 * It is the offsets that are queried and then combined with the robot pose to get
 * the current sensor orientation. It is this way around because the real robot has
 * no control over sensor orientation although the offsets are part of the robot design.
 *
 * NOTE: SHOULD the configuration file hold the robot setup in terms of the number
 *       of sensors and their initial placement? and should than be as individual
 *       items or an array. An array with named indices is more flexible
 *
 *
 *
 */
class RobotWallSensor {
 public:
  RobotWallSensor(sf::Vector2f origin = {0, 0}, float angle = 0, float half_angle = 5.0f, int ray_count = 16)
      : m_origin(origin), m_angle(angle), m_half_angle(half_angle), m_rays(ray_count + 1) {
    m_vertices.resize(m_rays);
    m_vertices.setPrimitiveType(sf::TriangleFan);
    m_vertices[0].position = m_origin;  // First vertex is the origin
    m_vertices[0].color = conf::SENSOR_COLOUR;

    m_max_range = conf::SENSOR_MAX_RANGE;
    m_power = 0.0f;
  }

  void set_origin(const sf::Vector2f& origin) {
    m_origin = origin;
    m_vertices[0].position = m_origin;  // Update origin vertex
  }

  /// The sensor fan will be centered on this angle and spread +/- the half-angle
  void set_angle(float angle) { m_angle = angle; }

  void set_half_angle(float half_angle) { m_half_angle = half_angle; }

  void set_ray_count(int ray_count) {
    m_rays = ray_count;
    m_vertices.resize(m_rays);
  }

  void SetGeometry(SensorGeometry geometry) { m_geometry = geometry; }

  SensorGeometry& getGeometry() { return m_geometry; }

  [[nodiscard]] float power() const { return m_power; }

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
    float startAngle = (m_angle - m_half_angle) * RADIANS;
    float endAngle = (m_angle + m_half_angle) * RADIANS;
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
      m_vertices[i].color.a = conf::SENSOR_ALPHA * (1.0f - closestHit / m_max_range);

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

  SensorGeometry m_geometry;
  sf::Vector2f m_origin = {0, 0};
  float m_angle;
  float m_half_angle;
  float m_max_range;
  int m_rays;
  float m_power;

  sf::VertexArray m_vertices;
};

#endif  // SENSOR_H
