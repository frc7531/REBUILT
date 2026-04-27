// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.zonecontroller;

// Composite zone design inspired by FRC Team 4481 (Team Rembrandts).
// Their 2026 build thread describes applying set theory (union, intersection,
// difference, complement) to combine primitive zones into complex field regions,
// similar to the Pathfinder tool in Adobe Illustrator.
// https://www.chiefdelphi.com/t/frc-4481-team-rembrandts-2026-build-thread-open-alliance/507296
// https://en.wikipedia.org/wiki/Intersection_(set_theory)

import edu.wpi.first.math.geometry.Translation2d;

public class CompositeZone implements Zone {

  public enum Operation {
    UNION,
    INTERSECTION,
    DIFFERENCE,
    COMPLEMENT
  }

  private final Zone a;
  private final Zone b;
  private Operation operation;

  private CompositeZone(Zone a, Zone b, Operation operation) {
    this.a = a;
    this.b = b;
    this.operation = operation;
  }

  @Override
  public boolean containsPoint(Translation2d point) {
    return switch (operation) {
      case UNION -> a.containsPoint(point) || b.containsPoint(point);
      case INTERSECTION -> a.containsPoint(point) && b.containsPoint(point);
      case DIFFERENCE -> a.containsPoint(point) && !b.containsPoint(point);
      case COMPLEMENT -> !a.containsPoint(point);
    };
  }

  /* A U B - inside A or inside B */
  public static CompositeZone union(Zone a, Zone b) {
    return new CompositeZone(a, b, Operation.UNION);
  }

  /* A ∩ B - inside A and inside B */
  public static CompositeZone intersection(Zone a, Zone b) {
    return new CompositeZone(a, b, Operation.INTERSECTION);
  }

  /* A \ B - inside A but not inside B */
  public static CompositeZone difference(Zone a, Zone b) {
    return new CompositeZone(a, b, Operation.DIFFERENCE);
  }

  /* ¬A - not inside A */
  public static CompositeZone complement(Zone a) {
    return new CompositeZone(a, null, Operation.COMPLEMENT);
  }
}
