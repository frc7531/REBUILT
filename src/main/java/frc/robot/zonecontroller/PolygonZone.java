// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.zonecontroller;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.Getter;

public class PolygonZone implements Zone {

  @Getter private final String name;
  @Getter private final Translation2d[] vertices;

  public PolygonZone(String name, Translation2d... vertices) {
    if (vertices.length < 3) {
      throw new IllegalArgumentException("A polygon requires at least 3 vertices");
    }

    this.name = name;
    this.vertices = vertices;
  }

  /*
   * Winding number algorithm — counts how many times the polygon winds
   * around the point. Non-zero = inside.
   */
  @Override
  public boolean containsPoint(Translation2d point) {
    int windingNumber = 0;
    int n = vertices.length;

    for (int i = 0; i < n; i++) {
      Translation2d v1 = vertices[i];
      Translation2d v2 = vertices[(i + 1) % n];

      if (v1.getY() <= point.getY()) {
        if (v2.getY() > point.getY()) {
          if (isLeft(v1, v2, point) > 0) windingNumber++;
        }
      } else {
        if (v2.getY() <= point.getY()) {
          if (isLeft(v1, v2, point) < 0) windingNumber--;
        }
      }
    }

    return windingNumber != 0;
  }

  /* Cross product of (v2-v1) and (point-v1). Positive = point is left of edge. */
  private static double isLeft(Translation2d v1, Translation2d v2, Translation2d point) {
    return (v2.getX() - v1.getX()) * (point.getY() - v1.getY())
        - (point.getX() - v1.getX()) * (v2.getY() - v1.getY());
  }
}
