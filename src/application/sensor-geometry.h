//
// Created by peter on 30/11/24.
//

#ifndef TYPES_H
#define TYPES_H

struct SensorGeometry {
  float x = 0;
  float y = 0;
  float theta = 0;
  float halfAngle = 5.0f;
  int rayCount = 32;
};

#endif  // TYPES_H
