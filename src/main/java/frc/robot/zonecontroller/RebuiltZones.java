// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.zonecontroller;

import frc.robot.FieldConstants;
import frc.robot.FieldConstants.LinesHorizontal;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.List;

public class RebuiltZones {

  public static final double TRENCH_BUFFER_M = 1.0;

  // --- Static zone instances (always blue-origin) ---

  public static final RectangleZone blueLeftTrench =
      new RectangleZone(
          "BlueLeftTrench",
          LinesVertical.hubCenter - TRENCH_BUFFER_M,
          LinesVertical.hubCenter + TRENCH_BUFFER_M,
          LinesHorizontal.leftTrenchOpenEnd,
          LinesHorizontal.leftTrenchOpenStart);

  public static final RectangleZone blueRightTrench =
      new RectangleZone(
          "BlueRightTrench",
          LinesVertical.hubCenter - TRENCH_BUFFER_M,
          LinesVertical.hubCenter + TRENCH_BUFFER_M,
          LinesHorizontal.rightTrenchOpenEnd,
          LinesHorizontal.rightTrenchOpenStart);

  public static final RectangleZone redLeftTrench = blueLeftTrench.mirrorForRedAlliance();
  public static final RectangleZone redRightTrench = blueRightTrench.mirrorForRedAlliance();

  public static final RectangleZone blueAllianceZone =
      new RectangleZone(
          "BlueAllianceZone", 0.0, LinesVertical.allianceZone, 0.0, FieldConstants.fieldWidth);

  public static final RectangleZone redAllianceZone = blueAllianceZone.mirrorForRedAlliance();

  public static final RectangleZone neutralZone =
      new RectangleZone(
          "NeutralZone",
          LinesVertical.neutralZoneNear,
          LinesVertical.neutralZoneFar,
          0.0,
          FieldConstants.fieldWidth);

  public static Zone allianceTrenches() {
    return AllianceFlipUtil.shouldFlip()
        ? redLeftTrench.union(redRightTrench)
        : blueLeftTrench.union(blueRightTrench);
  }

  public static Zone opponentTrenches() {
    return AllianceFlipUtil.shouldFlip()
        ? blueLeftTrench.union(blueRightTrench)
        : redLeftTrench.union(redRightTrench);
  }

  public static Zone allTrenches() {
    return blueLeftTrench.union(blueRightTrench).union(redLeftTrench).union(redRightTrench);
  }

  public static Zone allianceZone() {
    return AllianceFlipUtil.shouldFlip() ? redAllianceZone : blueAllianceZone;
  }

  public static Zone opponentAllianceZone() {
    return AllianceFlipUtil.shouldFlip() ? blueAllianceZone : redAllianceZone;
  }

  public static Zone neutralZone() {
    return neutralZone;
  }

  public static List<Zone> allZones() {
    return List.of(
        leftTrench,
        rightTrench,
        oppLeftTrench,
        oppRightTrench,
        neutralZoneLeft,
        neutralZoneRight,
        allianceZone,
        allianceZoneLeft,
        allianceZoneRight,
        oppAllianceZoneLeft,
        oppAllianceZoneRight);
  }

  // --- Backward compat statics for ZoneVisualizer ---
  public static final RectangleZone leftTrench = blueLeftTrench;
  public static final RectangleZone rightTrench = blueRightTrench;
  public static final RectangleZone oppLeftTrench = redLeftTrench;
  public static final RectangleZone oppRightTrench = redRightTrench;
  public static final RectangleZone neutralZoneLeft =
      new RectangleZone(
          "NeutralZoneLeft",
          LinesVertical.neutralZoneNear,
          LinesVertical.neutralZoneFar,
          LinesHorizontal.center,
          FieldConstants.fieldWidth);
  public static final RectangleZone neutralZoneRight =
      new RectangleZone(
          "NeutralZoneRight",
          LinesVertical.neutralZoneNear,
          LinesVertical.neutralZoneFar,
          0.0,
          LinesHorizontal.center);
  public static final RectangleZone allianceZone =
      new RectangleZone(
          "AllianceZone", 0.0, LinesVertical.allianceZone, 0.0, FieldConstants.fieldWidth);
  public static final RectangleZone allianceZoneLeft =
      new RectangleZone(
          "AllianceZoneLeft",
          0.0,
          LinesVertical.allianceZone,
          LinesHorizontal.center,
          FieldConstants.fieldWidth);
  public static final RectangleZone allianceZoneRight =
      new RectangleZone(
          "AllianceZoneRight", 0.0, LinesVertical.allianceZone, 0.0, LinesHorizontal.center);
  public static final RectangleZone oppAllianceZoneLeft = allianceZoneLeft.mirrorForRedAlliance();
  public static final RectangleZone oppAllianceZoneRight = allianceZoneRight.mirrorForRedAlliance();

  private RebuiltZones() {}
}
