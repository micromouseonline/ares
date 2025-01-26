#include <cmath>
#include <iostream>
#include "behaviour/trajectories/cubic.h"
#include "behaviour/trajectories/cubic_parameters.h"
#include "behaviour/trajectory.h"

struct PoseBasic {
  double x = 0;
  double y = 0;
  double theta = 0;
};

struct Velocity {
  double forward;
  double angular;
};

/// This function should work for any arbitrary version of CalculateVelocities
/// that return a v,w pair.
PoseBasic integratePath(double k) {
  Velocity vel;
  PoseBasic pose;
  float dt = 0.001f;
  Cubic cubic(194.81, 90, 1);  // Cubic trajectory with specified length, angle, and velocity
  cubic.setDeltaTime(dt);
  cubic.init(Pose());
  cubic.begin();

  int i = 0;
  while (!cubic.isFinished()) {
    cubic.update();

    vel.forward = cubic.getCurrentPose().getVelocity();
    vel.angular = cubic.getCurrentPose().getOmega() * RADIANS;

    // Calculate slip angle proportional to lateral acceleration
    double beta = k * vel.forward * vel.angular;

    // Adjust velocities for slip angle
    double adjusted_forward_velocity = vel.forward * cos(beta);

    // Integrate pose
    pose.x += adjusted_forward_velocity * cos(pose.theta) * dt + vel.forward * sin(beta) * sin(pose.theta) * dt;
    pose.y += adjusted_forward_velocity * sin(pose.theta) * dt - vel.forward * sin(beta) * cos(pose.theta) * dt;
    pose.theta += vel.angular * dt;  // Angle is corrected by gyro, no adjustment needed for slip

    printf("%3i, %8.3f,  %8.3f,  %8.3f, %8.3f,  %8.3f \n", i, vel.forward, vel.angular, pose.x, pose.y, pose.theta * 57.29);
    i++;
  }

  return pose;
}

int main() {
  double k = 0.00002;  // Proportionality constant for slip angle radians/(m/s^2)

  PoseBasic actualEndPose = integratePath(k);

  std::cout << "Actual End Pose: x = " << actualEndPose.x << ", y = " << actualEndPose.y << ", theta = " << actualEndPose.theta << std::endl;

  return 0;
}
