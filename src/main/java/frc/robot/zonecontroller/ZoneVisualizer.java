// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.zonecontroller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class ZoneVisualizer {

  private static final String logPrefix = "Zones/";
  private static final String avoidPrefix = "Zones/Avoid/";

  private static final int circleResolution = 32;

  private final Map<String, Pose2d[]> zoneOutlines = new HashMap<>();
  private final List<String> zoneNames = new ArrayList<>();

  public void addZone(RectangleZone zone) {
    register(zone.getName(), buildRectangle(zone));
  }

  public void addZone(CircleZone zone) {
    register(zone.getName(), buildCircle(zone));
  }

  public void addZone(PolygonZone zone) {
    register(zone.getName(), buildPolygon(zone));
  }

  public void addAvoidZone(AvoidZone avoidZone) {
    Zone inner = avoidZone.getZone();
    if (inner instanceof RectangleZone rect) {
      register("Avoid_" + avoidZone.getName(), buildRectangle(rect));
    } else if (inner instanceof CircleZone circle) {
      register("Avoid_" + avoidZone.getName(), buildCircle(circle));
    } else if (inner instanceof PolygonZone poly) {
      register("Avoid_" + avoidZone.getName(), buildPolygon(poly));
    }
  }

  public void addAllZones(List<Zone> zones, List<AvoidZone> avoidZones) {
    for (Zone zone : zones) {
      if (zone instanceof RectangleZone rect) {
        addZone(rect);
      } else if (zone instanceof CircleZone circle) {
        addZone(circle);
      } else if (zone instanceof PolygonZone poly) {
        addZone(poly);
      }
    }
    for (AvoidZone zone : avoidZones) {
      addAvoidZone(zone);
    }
  }

  public void addAvoidZones(List<AvoidZone> zones) {
    for (AvoidZone zone : zones) {
      addAvoidZone(zone);
    }
  }

  private void register(String name, Pose2d[] outline) {
    zoneNames.add(name);
    zoneOutlines.put(name, outline);
  }

  public void publishAll() {
    for (String name : zoneNames) {
      Logger.recordOutput(logPrefix + name, zoneOutlines.get(name));
    }
  }

  public void periodic() {
    publishAll();
  }

  public void periodic(Set<String> activeZoneNames) {
    for (String name : zoneNames) {
      Pose2d[] outline = zoneOutlines.get(name);

      Logger.recordOutput(logPrefix + name, outline);
      Logger.recordOutput(
          logPrefix + name + "/active", activeZoneNames.contains(name) ? outline : new Pose2d[0]);
    }
  }

  private static Pose2d[] buildRectangle(RectangleZone zone) {
    Rotation2d r = new Rotation2d();
    return new Pose2d[] {
      new Pose2d(zone.getXMin(), zone.getYMin(), r),
      new Pose2d(zone.getXMax(), zone.getYMin(), r),
      new Pose2d(zone.getXMax(), zone.getYMax(), r),
      new Pose2d(zone.getXMin(), zone.getYMax(), r),
      new Pose2d(zone.getXMin(), zone.getYMin(), r)
    };
  }

  private static Pose2d[] buildCircle(CircleZone zone) {
    Pose2d[] points = new Pose2d[circleResolution + 1];
    Translation2d center = zone.getCenter();
    double radius = zone.getRadius();
    Rotation2d r = new Rotation2d();

    for (int i = 0; i < circleResolution; i++) {
      double angle = 2.0 * Math.PI * i / circleResolution;
      points[i] =
          new Pose2d(
              center.getX() + radius * Math.cos(angle),
              center.getY() + radius * Math.sin(angle),
              r);
    }
    points[circleResolution] = points[0];
    return points;
  }

  private static Pose2d[] buildPolygon(PolygonZone zone) {
    Translation2d[] vertices = zone.getVertices();
    Pose2d[] points = new Pose2d[vertices.length + 1];
    Rotation2d r = new Rotation2d();

    for (int i = 0; i < vertices.length; i++) {
      points[i] = new Pose2d(vertices[i], r);
    }

    points[vertices.length] = points[0];
    return points;
  }
}
