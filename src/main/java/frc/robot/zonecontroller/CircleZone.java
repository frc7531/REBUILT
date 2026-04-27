// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.zonecontroller;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.Getter;

public class CircleZone implements Zone {

  @Getter private final String name;
  @Getter private final Translation2d center;
  @Getter private final double radius;

  public CircleZone(String name, Translation2d center, double radius) {
    this.name = name;
    this.center = center;
    this.radius = radius;
  }

  @Override
  public boolean containsPoint(Translation2d point) {
    return point.getDistance(center) <= radius;
  }
}
