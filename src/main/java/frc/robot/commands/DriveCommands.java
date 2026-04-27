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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {

  private static final double DEADBAND = 0.1;
  private static final LoggedTunableNumber aimKP = new LoggedTunableNumber("Drive/AimKP", 12.0);
  private static final LoggedTunableNumber aimKD = new LoggedTunableNumber("Drive/AimKD", 0.15);
  private static final LoggedTunableNumber aimOmegaCap =
      new LoggedTunableNumber("Drive/AimOmegaCap", 8.0);
  private static final LoggedTunableNumber manualKP =
      new LoggedTunableNumber("Drive/ManualKP", 5.0);
  private static final LoggedTunableNumber manualKD =
      new LoggedTunableNumber("Drive/ManualKD", 0.4);

  private static final double FF_START_DELAY = 2.0;
  private static final double FF_RAMP_RATE = 0.1;
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05;

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double mag = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d dir = new Rotation2d(Math.atan2(y, x));
    mag = mag * mag;
    return new Pose2d(Translation2d.kZero, dir)
        .transformBy(new Transform2d(mag, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Optional<Rotation2d>> headingOverride) {

    PIDController angleController = new PIDController(manualKP.get(), 0, manualKD.get());
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    final boolean[] wasAiming = {false};

    return Commands.run(
            () -> {
              Translation2d linear =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              double joystickOmega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
              joystickOmega = Math.copySign(joystickOmega * joystickOmega, joystickOmega);
              joystickOmega *= drive.getMaxAngularSpeedRadPerSec();

              Optional<Rotation2d> override = headingOverride.get();
              boolean aiming = override.isPresent();

              // Set drive goal based on aiming state
              if (aiming) {
                drive.setGoal(Drive.Goal.AIM_AND_DRIVE);
              } else {
                drive.setGoal(Drive.Goal.TELEOP);
              }

              angleController.setP(aiming ? aimKP.get() : manualKP.get());
              angleController.setD(aiming ? aimKD.get() : manualKD.get());

              if (aiming && !wasAiming[0]) angleController.reset();
              wasAiming[0] = aiming;

              double omega;
              if (aiming) {
                omega =
                    angleController.calculate(
                        RobotState.getInstance().getRotation().getRadians(),
                        override.get().getRadians());
                omega = MathUtil.clamp(omega, -aimOmegaCap.get(), aimOmegaCap.get());
              } else {
                omega = joystickOmega;
              }

              // Build speeds at full scale — Drive.runVelocity applies the multiplier
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linear.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linear.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? RobotState.getInstance().getRotation().plus(new Rotation2d(Math.PI))
                          : RobotState.getInstance().getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              angleController.reset();
              wasAiming[0] = false;
              drive.setGoal(Drive.Goal.TELEOP);
            })
        .finallyDo(() -> drive.setGoal(Drive.Goal.TELEOP));
  }

  public static Command joystickDrive(
      Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return joystickDrive(drive, x, y, omega, Optional::empty);
  }

  public static Command aimAtTarget(Drive drive) {
    ProfiledPIDController rotationPID =
        new ProfiledPIDController(
            aimKP.get(),
            0.0,
            aimKD.get(),
            new TrapezoidProfile.Constraints(
                drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec() * 2.0));
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    rotationPID.setTolerance(Units.degreesToRadians(2.0));

    return Commands.run(
            () -> {
              Rotation2d targetHeading = AimUtil.calculateHeading();
              double omega =
                  rotationPID.calculate(
                      RobotState.getInstance().getRotation().getRadians(),
                      targetHeading.getRadians());
              drive.runVelocity(new ChassisSpeeds(0, 0, omega));
            })
        .beforeStarting(
            () -> rotationPID.reset(RobotState.getInstance().getRotation().getRadians()));
  }

  // --- Characterization ---

  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  velocitySamples.clear();
                  voltageSamples.clear();
                  drive.setGoal(Drive.Goal.CHARACTERIZE);
                }),
            Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(FF_START_DELAY),
            Commands.runOnce(timer::restart),
            Commands.run(
                    () -> {
                      double voltage = timer.get() * FF_RAMP_RATE;
                      drive.runCharacterization(voltage);
                      velocitySamples.add(drive.getFFCharacterizationVelocity());
                      voltageSamples.add(voltage);
                    },
                    drive)
                .finallyDo(
                    () -> {
                      int n = velocitySamples.size();
                      double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
                      for (int i = 0; i < n; i++) {
                        sumX += velocitySamples.get(i);
                        sumY += voltageSamples.get(i);
                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                      }
                      double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                      double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                      NumberFormat fmt = new DecimalFormat("#0.00000");
                      System.out.println("********** Drive FF Results **********");
                      System.out.println("\tkS: " + fmt.format(kS));
                      System.out.println("\tkV: " + fmt.format(kV));
                    }))
        .finallyDo(() -> drive.setGoal(Drive.Goal.TELEOP));
  }

  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    double[][] stateArr = {new double[4], {0}, {0}};
    Rotation2d[] lastAngle = {Rotation2d.kZero};

    return Commands.parallel(
            Commands.sequence(
                Commands.runOnce(
                    () -> {
                      limiter.reset(0.0);
                      drive.setGoal(Drive.Goal.CHARACTERIZE);
                    }),
                Commands.run(
                    () ->
                        drive.runVelocity(
                            new ChassisSpeeds(0, 0, limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY))),
                    drive)),
            Commands.sequence(
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                    () -> {
                      stateArr[0] = drive.getWheelRadiusCharacterizationPositions();
                      lastAngle[0] = RobotState.getInstance().getRotation();
                      stateArr[2][0] = 0;
                    }),
                Commands.run(
                        () -> {
                          var rot = RobotState.getInstance().getRotation();
                          stateArr[2][0] += Math.abs(rot.minus(lastAngle[0]).getRadians());
                          lastAngle[0] = rot;
                        })
                    .finallyDo(
                        () -> {
                          double[] positions = drive.getWheelRadiusCharacterizationPositions();
                          double delta = 0;
                          for (int i = 0; i < 4; i++)
                            delta += Math.abs(positions[i] - stateArr[0][i]) / 4.0;
                          double radius = (stateArr[2][0] * Drive.DRIVE_BASE_RADIUS) / delta;
                          NumberFormat fmt = new DecimalFormat("#0.000");
                          System.out.println("********** Wheel Radius Results **********");
                          System.out.println(
                              "\tRadius: "
                                  + fmt.format(radius)
                                  + " m, "
                                  + fmt.format(Units.metersToInches(radius))
                                  + " in");
                        })))
        .finallyDo(() -> drive.setGoal(Drive.Goal.TELEOP));
  }
}
