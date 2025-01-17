#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include "SFML/Graphics.hpp"
#include "collisions.h"
#include "configuration.h"
#include "drawing.h"
#include "utils.h"
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
  RobotWallSensor(sf::Vector2f origin = {0, 0}, float angle = 0) : m_origin(origin), m_angle(angle) {
    m_vertices.resize(m_geometry.rayCount);
    m_vertices.setPrimitiveType(sf::TriangleFan);
    m_vertices[0].position = m_origin;  // First vertex is the origin
    m_vertices[0].color = conf::SENSOR_COLOUR;
    m_max_range = conf::SENSOR_MAX_RANGE;
    m_power = 0.0f;
  }

  void setOrigin(const sf::Vector2f& origin) {
    m_origin = origin;
    m_vertices[0].position = m_origin;  // Update origin vertex
  }

  /// The sensor fan will be centered on this angle and spread +/- the half-angle
  void setAngle(float angle) {
    m_angle = angle;  //
  }

  void setHalfAngle(float half_angle) {
    m_geometry.halfAngle = half_angle;  //
  }

  void setRayCount(int ray_count) {
    m_geometry.rayCount = ray_count;
    m_vertices.resize(ray_count);
  }

  void SetGeometry(SensorGeometry geometry) {
    m_geometry = geometry;
    m_vertices.resize(geometry.rayCount);
  }

  SensorGeometry& getGeometry() {
    return m_geometry;  //
  }

  [[nodiscard]] float getDistance() const {
    return m_distance;  //
  }

  [[nodiscard]] float getPower() const {
    return m_power;  //
  }

  float getPowerFromDistance(float distance) {
    m_power = 1600.0f * expf(-0.025f * distance);  //
    return m_power;
  }

  /***
   * generate a complete sensor fan for every rectangle in the supplied
   * vector. This is surprisingly fast. Even so, in the actual simulation
   * the rectangles list passed in will only be those surrounding cells.
   * The eight immediate neighbours should be enough so a maximum of
   * 40 rectangles is required, including the posts
   * @param obstacles
   */
  void update(const std::vector<sf::FloatRect>& obstacles) {
    // Calculate angular increment for rays
    float startAngle = (m_angle - m_geometry.halfAngle) * RADIANS;
    float endAngle = (m_angle + m_geometry.halfAngle) * RADIANS;
    float angleIncrement = (endAngle - startAngle) / float(m_geometry.rayCount - 1);

    float total_distance = 0;
    for (int i = 1; i < m_geometry.rayCount; ++i) {  // Remember to skip origin (index 0)
      float angle = startAngle + float(i - 1) * angleIncrement;
      sf::Vector2f dir = {std::cos(angle), std::sin(angle)};

      float closestHit = m_max_range;
      for (const auto& rect : obstacles) {
        float distance = Collisions::getRayDistanceToAlignedRectangle(m_origin, dir, rect, m_max_range);
        if (distance < closestHit) {
          closestHit = distance;
        }
      }

      // Update the ray endpoint
      sf::Vector2f hitPosition = m_origin + dir * closestHit;
      m_vertices[i].position = hitPosition;
      m_vertices[i].color.a = uint8_t(conf::SENSOR_ALPHA * (1.0f - closestHit / m_max_range));

      /// Accumulate distance and power for averaging
      total_distance += closestHit;
    }
    m_distance = total_distance / (float)m_geometry.rayCount;
    m_power = getPowerFromDistance(m_distance);
  }

  /// The sensor will be drawn as a triangle fan. This is really
  /// fast because the GPU does all the work from the vertex list
  void draw(sf::RenderTarget& renderTarget) const {
    renderTarget.draw(m_vertices);  //
  }

 private:
  SensorGeometry m_geometry = {.x = 0, .y = 0, .theta = 0, .halfAngle = 5.0f, .rayCount = 16};
  sf::Vector2f m_origin = {0, 0};
  float m_angle;
  float m_max_range;
  float m_distance;
  float m_power;

  sf::VertexArray m_vertices;
};

#endif  // SENSOR_H
