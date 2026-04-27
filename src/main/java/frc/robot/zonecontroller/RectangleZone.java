// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.zonecontroller;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import lombok.Getter;

public class RectangleZone implements Zone {

  @Getter private final String name;
  @Getter private final double xMin, xMax, yMin, yMax;

  public RectangleZone(String name, double xMin, double xMax, double yMin, double yMax) {
    this.name = name;
    this.xMin = xMin;
    this.xMax = xMax;
    this.yMin = yMin;
    this.yMax = yMax;
  }

  @Override
  public boolean containsPoint(Translation2d point) {
    return point.getX() >= xMin
        && point.getX() <= xMax
        && point.getY() >= yMin
        && point.getY() <= yMax;
  }

  public RectangleZone mirrorForRedAlliance() {
    return new RectangleZone(
        name + "_red",
        FieldConstants.fieldLength - xMin,
        FieldConstants.fieldLength - xMax,
        yMin,
        yMax);
  }
}
