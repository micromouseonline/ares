//
// Created by peter on 02/02/25.
//

/******************************************************************************
 * Project: mr32-ares                                                         *
 * -----                                                                      *
 * Copyright 2022 - 2025 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#pragma once

/***
 * Stub for an on-board odometry system
 *
 * Normally this would use the motor encoders to calculate odometry
 *
 * It would be better perhaps to combine encoders and IMU into a single pose
 * estimation system
 */

class Odometry {
 public:
  void begin() {
  }

  void reset() {
  }

  void resetLeft() {
  }

  void resetRight() {
  }

  float angle() {
  }

  float omega() {
  }

  float distance() {
  }

  float velocity() {
  }

  int leftCount() {
  }

  int rightCount() {
  }

  void update() {
    /// caled from systick
  }

 private:
};
