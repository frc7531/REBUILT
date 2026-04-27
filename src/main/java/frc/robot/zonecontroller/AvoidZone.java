// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.zonecontroller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Hub;
import frc.robot.FieldConstants.LinesHorizontal;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.FieldConstants.Tower;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;

public class AvoidZone {

  @Getter private final String name;
  @Getter private final Zone zone;
  @Getter private final double margin;

  private final Zone expanded;

  public AvoidZone(String name, Zone zone, double margin) {
    this.name = name;
    this.zone = zone;
    this.margin = margin;

    if (zone instanceof RectangleZone rect) {
      this.expanded =
          new RectangleZone(
              name + "_expanded",
              rect.getXMin() - margin,
              rect.getXMax() + margin,
              rect.getYMin() - margin,
              rect.getYMax() + margin);
    } else if (zone instanceof CircleZone circle) {
      this.expanded =
          new CircleZone(name + "_expanded", circle.getCenter(), circle.getRadius() + margin);
    } else {
      this.expanded = zone;
    }
  }

  public AvoidZone(String name, Zone zone) {
    this(name, zone, 0.0);
  }

  public boolean isInInfluenceZone(Translation2d point) {
    return expanded.containsPoint(point);
  }

  public boolean contains(Translation2d point) {
    return zone.containsPoint(point);
  }

  public Translation2d getRepulsiveVelocity(
      Translation2d robotPos, double influenceRadius, double maxRepulsionMPS) {
    if (!isNearZone(robotPos, influenceRadius)) {
      return Translation2d.kZero;
    }

    Translation2d nearest = getNearestBoundaryPoint(robotPos);
    double dist = robotPos.getDistance(nearest);

    if (zone.containsPoint(robotPos)) {
      dist = 0.0;
    }

    if (dist >= influenceRadius) {
      return Translation2d.kZero;
    }

    double magnitude;
    if (dist < 1e-4) {
      magnitude = maxRepulsionMPS;
    } else {
      magnitude = maxRepulsionMPS * (1.0 - (dist / influenceRadius));
    }
    magnitude = MathUtil.clamp(magnitude, 0, maxRepulsionMPS);

    Translation2d away = robotPos.minus(nearest);
    double norm = away.getNorm();
    if (norm < 1e-6) {
      return new Translation2d(magnitude, 0);
    }
    return away.times(magnitude / norm);
  }

  private boolean isNearZone(Translation2d point, double influenceRadius) {
    if (zone instanceof RectangleZone rect) {
      return point.getX() >= rect.getXMin() - influenceRadius
          && point.getX() <= rect.getXMax() + influenceRadius
          && point.getY() >= rect.getYMin() - influenceRadius
          && point.getY() <= rect.getYMax() + influenceRadius;
    } else if (zone instanceof CircleZone circle) {
      return point.getDistance(circle.getCenter()) <= circle.getRadius() + influenceRadius;
    }
    return expanded.containsPoint(point) || zone.containsPoint(point);
  }

  private Translation2d getNearestBoundaryPoint(Translation2d point) {
    if (zone instanceof RectangleZone rect) {
      double cx = MathUtil.clamp(point.getX(), rect.getXMin(), rect.getXMax());
      double cy = MathUtil.clamp(point.getY(), rect.getYMin(), rect.getYMax());
      return new Translation2d(cx, cy);
    } else if (zone instanceof CircleZone circle) {
      Translation2d toPoint = point.minus(circle.getCenter());
      double norm = toPoint.getNorm();
      if (norm < 1e-6) return circle.getCenter().plus(new Translation2d(circle.getRadius(), 0));
      return circle.getCenter().plus(toPoint.times(circle.getRadius() / norm));
    }
    return point;
  }

  // Buffer = robot half-width only — just enough so the robot center doesn't enter the zone
  private static final double BUFFER = 0.45;

  public static final AvoidZone ALLIANCE_HUB =
      new AvoidZone(
          "AllianceHub",
          new RectangleZone(
              "AllianceHub",
              Hub.nearRightCorner.getX() - BUFFER,
              Hub.farRightCorner.getX() + BUFFER,
              Hub.nearRightCorner.getY() - BUFFER,
              Hub.nearLeftCorner.getY() + BUFFER));

  public static final AvoidZone OPP_HUB =
      new AvoidZone(
          "OpponentHub",
          new RectangleZone(
              "OpponentHub",
              Hub.oppNearRightCorner.getX() - BUFFER,
              Hub.oppFarRightCorner.getX() + BUFFER,
              Hub.oppNearRightCorner.getY() - BUFFER,
              Hub.oppNearLeftCorner.getY() + BUFFER));

  public static final AvoidZone ALLIANCE_TOWER =
      new AvoidZone(
          "AllianceTower",
          new RectangleZone(
              "AllianceTower",
              Tower.frontFaceX - Tower.depth - BUFFER,
              Tower.frontFaceX + BUFFER,
              Tower.rightUpright.getY() - BUFFER,
              Tower.leftUpright.getY() + BUFFER));

  public static final AvoidZone OPP_TOWER =
      new AvoidZone(
          "OpponentTower",
          new RectangleZone(
              "OpponentTower",
              FieldConstants.fieldLength - Tower.frontFaceX - BUFFER,
              FieldConstants.fieldLength - Tower.frontFaceX + Tower.depth + BUFFER,
              Tower.oppRightUpright.getY() - BUFFER,
              Tower.oppLeftUpright.getY() + BUFFER));

  public static final AvoidZone LEFT_BUMP =
      new AvoidZone(
          "LeftBump",
          new RectangleZone(
              "LeftBump",
              LinesVertical.hubCenter - FieldConstants.LeftBump.depth / 2.0 - BUFFER,
              LinesVertical.hubCenter + FieldConstants.LeftBump.depth / 2.0 + BUFFER,
              LinesHorizontal.leftBumpEnd - BUFFER,
              LinesHorizontal.leftBumpStart + BUFFER));

  public static final AvoidZone RIGHT_BUMP =
      new AvoidZone(
          "RightBump",
          new RectangleZone(
              "RightBump",
              LinesVertical.hubCenter - FieldConstants.RightBump.depth / 2.0 - BUFFER,
              LinesVertical.hubCenter + FieldConstants.RightBump.depth / 2.0 + BUFFER,
              LinesHorizontal.rightBumpEnd - BUFFER,
              LinesHorizontal.rightBumpStart + BUFFER));

  public static List<AvoidZone> defaultZones() {
    List<AvoidZone> zones = new ArrayList<>();
    zones.add(ALLIANCE_HUB);
    zones.add(OPP_HUB);
    zones.add(ALLIANCE_TOWER);
    zones.add(OPP_TOWER);
    return zones;
  }

  public static List<AvoidZone> allStructures() {
    List<AvoidZone> zones = new ArrayList<>(defaultZones());
    zones.add(LEFT_BUMP);
    zones.add(RIGHT_BUMP);
    return zones;
  }

  @Override
  public String toString() {
    return name;
  }
}
