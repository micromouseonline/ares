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
Pose integratePath(double k) {
  Velocity vel;
  //  PoseBasic pose;
  Pose p;
  float dt = 0.001f;
  Cubic cubic(194.81, 90, 1000);  // Cubic trajectory with specified length, angle, and velocity
  cubic.setDeltaTime(dt);
  cubic.init(p);
  cubic.begin();

  int i = 0;
  while (!cubic.isFinished()) {
    cubic.updateWithSlip(k);

    p = cubic.getCurrentPose();
    vel.forward = cubic.getCurrentPose().getVelocity();
    vel.angular = cubic.getCurrentPose().getOmega() * RADIANS;
    p.print();

    //    printf("%3i, %8.3f,  %8.3f,  %8.3f, %8.3f,  %8.3f \n", i, vel.forward, vel.angular, p.getX(), p.getY(), p.getAngle());
    i++;
  }

  return p;
}

int main() {
  double k = 0.1;  // Proportionality constant for slip angle deg/(m/s^2)

  Pose endPose = integratePath(k);
  endPose.print();

  return 0;
}
