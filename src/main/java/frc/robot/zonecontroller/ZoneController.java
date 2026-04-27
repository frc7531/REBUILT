// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.zonecontroller;

import static frc.robot.FieldConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.ShotMode;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class ZoneController extends VirtualSubsystem {

  public enum ZoneAction {
    SHOOT_HUB,
    FERRY_LEFT,
    FERRY_RIGHT,
    STOW_HOOD
  }

  public enum CurrentZone {
    ALLIANCE,
    NEUTRAL_LEFT,
    NEUTRAL_RIGHT,
    OPPONENT_LEFT,
    OPPONENT_RIGHT,
    TRENCH,
    UNKNOWN
  }

  public static final Translation2d FERRY_LEFT_TARGET =
      new Translation2d(LinesVertical.allianceZone / 2.0, (fieldWidth / 2.0 + fieldWidth) / 2.0);
  public static final Translation2d FERRY_RIGHT_TARGET =
      new Translation2d(LinesVertical.allianceZone / 2.0, (fieldWidth / 2.0 + 0.0) / 2.0);

  private CurrentZone currentZone = CurrentZone.UNKNOWN;
  private ZoneAction action = ZoneAction.SHOOT_HUB;

  private static ZoneController instance;

  public static ZoneController getInstance() {
    if (instance == null) instance = new ZoneController();
    return instance;
  }

  private ZoneController() {}

  @Override
  public void periodic() {
    Translation2d pos = RobotState.getInstance().getEstimatedPose().getTranslation();
    currentZone = classifyZone(pos);
    action = applyOverrides(determineAction(currentZone));

    Logger.recordOutput("ZoneController/Zone", currentZone.toString());
    Logger.recordOutput("ZoneController/Action", action.toString());
    Logger.recordOutput("ZoneController/AimTarget", getAimTarget());
  }

  private CurrentZone classifyZone(Translation2d pos) {
    if (RebuiltZones.allTrenches().containsPoint(pos)) return CurrentZone.TRENCH;
    if (RebuiltZones.allianceZone().containsPoint(pos)) return CurrentZone.ALLIANCE;
    if (RebuiltZones.neutralZoneLeft.containsPoint(pos)) return CurrentZone.NEUTRAL_LEFT;
    if (RebuiltZones.neutralZoneRight.containsPoint(pos)) return CurrentZone.NEUTRAL_RIGHT;
    if (RebuiltZones.oppAllianceZoneLeft.containsPoint(pos)) return CurrentZone.OPPONENT_LEFT;
    if (RebuiltZones.oppAllianceZoneRight.containsPoint(pos)) return CurrentZone.OPPONENT_RIGHT;
    return CurrentZone.UNKNOWN;
  }

  public ZoneAction determineAction(CurrentZone zone) {
    return switch (zone) {
      case ALLIANCE -> ZoneAction.SHOOT_HUB;
      case NEUTRAL_LEFT, OPPONENT_LEFT -> ZoneAction.FERRY_LEFT;
      case NEUTRAL_RIGHT, OPPONENT_RIGHT -> ZoneAction.FERRY_RIGHT;
      case TRENCH -> ZoneAction.STOW_HOOD;
      case UNKNOWN -> ZoneAction.SHOOT_HUB;
    };
  }

  private ZoneAction applyOverrides(ZoneAction zoneAction) {
    ShotMode mode = RobotState.getInstance().getShotMode();
    if (mode == ShotMode.FORCE_HUB) return ZoneAction.SHOOT_HUB;
    if (mode == ShotMode.FORCE_FERRY) {
      Translation2d pos = RobotState.getInstance().getEstimatedPose().getTranslation();
      return pos.getY() > FieldConstants.fieldWidth / 2.0
          ? ZoneAction.FERRY_LEFT
          : ZoneAction.FERRY_RIGHT;
    }
    return zoneAction;
  }

  public ZoneAction getAction() {
    return action;
  }

  public CurrentZone getCurrentZone() {
    return currentZone;
  }

  public boolean shouldFerry() {
    return action == ZoneAction.FERRY_LEFT || action == ZoneAction.FERRY_RIGHT;
  }

  public boolean shouldStowHood() {
    return action == ZoneAction.STOW_HOOD;
  }

  public Translation2d getAimTarget() {
    return switch (action) {
      case SHOOT_HUB, STOW_HOOD -> getHubCenter();
      case FERRY_LEFT -> AllianceFlipUtil.apply(FERRY_LEFT_TARGET);
      case FERRY_RIGHT -> AllianceFlipUtil.apply(FERRY_RIGHT_TARGET);
    };
  }

  public Translation2d getHubCenter() {
    return AllianceFlipUtil.shouldFlip()
        ? FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()
        : FieldConstants.Hub.topCenterPoint.toTranslation2d();
  }
}
