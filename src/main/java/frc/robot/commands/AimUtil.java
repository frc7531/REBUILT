// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AimUtil {

  private static final LoggedTunableNumber sotmThreshold =
      new LoggedTunableNumber("Aim/SOTMThresholdMPS", 0.5);
  private static final LoggedTunableNumber fuelSpeed =
      new LoggedTunableNumber("Aim/FuelSpeedMPS", 12.0);
  private static final LoggedTunableNumber releaseDelay =
      new LoggedTunableNumber("Aim/ReleaseDelaySec", 0.20);
  private static final LoggedTunableNumber maxLookahead =
      new LoggedTunableNumber("Aim/MaxLookaheadSec", 1.0);

  private AimUtil() {}

  public static Rotation2d calculateHeading() {
    RobotState state = RobotState.getInstance();
    Translation2d robot = state.getEstimatedPose().getTranslation();
    Translation2d target = state.getAimTarget();
    ChassisSpeeds velocity = state.getFieldVelocity();

    double speed = Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);

    Rotation2d heading;
    if (speed < sotmThreshold.get()) {
      heading = target.minus(robot).getAngle();
    } else {
      double distance = robot.getDistance(target);
      double tof = distance / Math.max(fuelSpeed.get(), 0.1);
      double totalTime = MathUtil.clamp(releaseDelay.get() + tof, 0.0, maxLookahead.get());

      Translation2d future =
          robot.plus(
              new Translation2d(
                  velocity.vxMetersPerSecond * totalTime, velocity.vyMetersPerSecond * totalTime));
      heading = target.minus(future).getAngle();
      Logger.recordOutput("Aim/PredictedPos", future);
    }

    Logger.recordOutput("Aim/Speed", speed);
    Logger.recordOutput("Aim/UsingSOTM", speed >= sotmThreshold.get());
    Logger.recordOutput("Aim/Heading", heading);
    Logger.recordOutput("Aim/Target", target);
    return heading;
  }
}
