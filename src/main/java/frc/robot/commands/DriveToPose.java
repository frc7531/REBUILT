// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* Stolen from 6328, idek why. I never used it. Probably could have though*/
public class DriveToPose extends Command {

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("DriveToPose/kP", 3.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("DriveToPose/kD", 0.05);
  private static final LoggedTunableNumber maxSpeed =
      new LoggedTunableNumber("DriveToPose/MaxSpeedMPS", 3.5);
  private static final LoggedTunableNumber thetaKP =
      new LoggedTunableNumber("DriveToPose/ThetakP", 5.0);
  private static final LoggedTunableNumber thetaKD =
      new LoggedTunableNumber("DriveToPose/ThetakD", 0.1);
  private static final LoggedTunableNumber thetaMaxVel =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelRadPS", 6.0);
  private static final LoggedTunableNumber thetaMaxAccel =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAccelRadPSS", 8.0);
  private static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("DriveToPose/ToleranceMeters", 0.05);
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/ThetaToleranceDeg", 2.0);
  private static final LoggedTunableNumber slowdownRadius =
      new LoggedTunableNumber("DriveToPose/SlowdownMeters", 0.5);

  private final Drive drive;
  private final Supplier<Pose2d> target;

  private final PIDController xPID = new PIDController(0, 0, 0);
  private final PIDController yPID = new PIDController(0, 0, 0);
  private final ProfiledPIDController thetaPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private Pose2d goalPose;

  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  public DriveToPose(Drive drive, Pose2d target) {
    this(drive, () -> AllianceFlipUtil.apply(target));
  }

  @Override
  public void initialize() {
    goalPose = target.get();
    xPID.setPID(kP.get(), 0, kD.get());
    yPID.setPID(kP.get(), 0, kD.get());
    xPID.reset();
    yPID.reset();
    thetaPID.setPID(thetaKP.get(), 0, thetaKD.get());
    thetaPID.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVel.get(), thetaMaxAccel.get()));
    Pose2d current = RobotState.getInstance().getEstimatedPose();
    thetaPID.reset(current.getRotation().getRadians());
    drive.setGoal(Drive.Goal.AUTO_PATH);
  }

  @Override
  public void execute() {
    goalPose = target.get();
    Pose2d current = RobotState.getInstance().getEstimatedPose();

    double xOut = xPID.calculate(current.getX(), goalPose.getX());
    double yOut = yPID.calculate(current.getY(), goalPose.getY());

    double dist = current.getTranslation().getDistance(goalPose.getTranslation());
    double max = maxSpeed.get() * MathUtil.clamp(dist / slowdownRadius.get(), 0.05, 1.0);
    double mag = Math.hypot(xOut, yOut);
    if (mag > max) {
      xOut *= max / mag;
      yOut *= max / mag;
    }

    double thetaOut =
        thetaPID.calculate(current.getRotation().getRadians(), goalPose.getRotation().getRadians());
    thetaOut =
        MathUtil.clamp(
            thetaOut, -drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(xOut, yOut, thetaOut, current.getRotation()));

    Logger.recordOutput("DriveToPose/Goal", goalPose);
    Logger.recordOutput("DriveToPose/Error", dist);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    Pose2d current = RobotState.getInstance().getEstimatedPose();
    double dist = current.getTranslation().getDistance(goalPose.getTranslation());
    double thetaErr =
        Math.abs(
            MathUtil.angleModulus(
                current.getRotation().getRadians() - goalPose.getRotation().getRadians()));
    return dist < tolerance.get() && thetaErr < Math.toRadians(thetaTolerance.get());
  }
}
