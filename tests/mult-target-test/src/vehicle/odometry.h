/***
 * Provide a generic interface for robot odometry
 *
 * By consuming data from a pair of wheel encoders and an IMU, calculate the
 * robot's position and orientation as well as its forward and rotational velocity.
 *
 * Need to add a way to inject the dependencies for the encoders and imu.
 *
 * There needs to be a way to do this both with and without an IMU.
 *
 * Odometry provides (all in natural units):
 *   Distance - Total
 *   Distance - Relative
 *   Angle - World
 *   Angle - Relative
 *   Forward Velocity
 *   Angular velocity
 *   x
 *   y
 *
 *  All distances are integrated from forward velocity
 *  All angles are integrated from angular velocity
 *
 *  (X,Y) are world coordinated calculated from forward velocity and world angle.
 *
 */
#pragma once

#include <cmath>

#include "hal/encoder-hal.h"
#include "hal/imu-hal.h"

const float RADIANS = 180.0 / M_PI;

class Odometry {
 public:
  Odometry(IMUHAL& imu, EncoderHAL& leftEncoder, EncoderHAL& rightEncoder)
      : imu(imu),
        leftEncoder(leftEncoder),
        rightEncoder(rightEncoder) {
  }

  void update(float delta_time) {
    // Read encoder values
    /// these are assumed to be deltas
    int leftTicks = leftEncoder.getCount();
    int rightTicks = rightEncoder.getCount();

    // Read IMU heading
    m_angular_velocity = imu.getYawRate();
    m_heading += m_angular_velocity * delta_time;

    // Compute position and speed (example calculation)
    float leftDistance = leftTicks * TICKS_TO_METERS;
    float rightDistance = rightTicks * TICKS_TO_METERS;
    float deltaDistance = (leftDistance + rightDistance) / 2.0;

    posX += deltaDistance * cos(m_heading * RADIANS);
    posY += deltaDistance * sin(m_heading * RADIANS);
    m_distance += deltaDistance;
    velocity = deltaDistance / delta_time;
  }

  float getX() const {
    return posX;
  }
  float getY() const {
    return posY;
  }
  float getVelocity() const {
    return velocity;
  }

 private:
  IMUHAL& imu;
  EncoderHAL& leftEncoder;
  EncoderHAL& rightEncoder;

  float posX = 0.0;
  float posY = 0.0;
  float velocity = 0.0;
  float m_angular_velocity = 0.0;
  float m_heading = 0.0f;
  float m_distance = 0;

  static constexpr float TICKS_TO_METERS = 0.001;  // Example conversion
  static constexpr float DT = 0.01;                // Example time step (10ms)
};
