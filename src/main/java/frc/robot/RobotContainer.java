// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Mode;
import frc.robot.RobotState.ShotMode;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.oi.Controls;
import frc.robot.oi.ControlsXbox;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.AutoChooser;
import frc.robot.util.geometry.AllianceFlipUtil;
import frc.robot.zonecontroller.AvoidZone;
import frc.robot.zonecontroller.RebuiltZones;
import frc.robot.zonecontroller.ZoneController;
import frc.robot.zonecontroller.ZoneVisualizer;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private Drive drive;
  private Vision vision;

  private final ZoneVisualizer zoneVisualizer = new ZoneVisualizer();
  private final Controls controls;
  private final AutoChooser chooser;

  private Optional<Rotation2d> headingOverride = Optional.empty();

  private final LoggedDashboardChooser<ShotMode> shotModeChooser;

  public RobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOLimelight("limelight-front", drive::getRotation));
        }
        case PRACBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOLimelight("limelight-front", drive::getRotation));
        }
        case SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(TunerConstants.FrontLeft),
                  new ModuleIOSim(TunerConstants.FrontRight),
                  new ModuleIOSim(TunerConstants.BackLeft),
                  new ModuleIOSim(TunerConstants.BackRight));
        }
      }
    }

    // No-op fallbacks
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      vision = new Vision(drive::addVisionMeasurement, new VisionIO[] {});
    }

    // Shot mode dashboard chooser
    shotModeChooser = new LoggedDashboardChooser<>("Shot Mode");
    shotModeChooser.addDefaultOption("Auto", ShotMode.AUTO);
    shotModeChooser.addOption("Force Hub", ShotMode.FORCE_HUB);
    shotModeChooser.addOption("Force Ferry", ShotMode.FORCE_FERRY);

    if (Constants.tuningMode) {
      zoneVisualizer.addAllZones(RebuiltZones.allZones(), AvoidZone.allStructures());
    }

    controls = new ControlsXbox(0);
    chooser = new AutoChooser(drive);
    configureBindings();
  }

  private void configureBindings() {
    // Drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            controls::getForward,
            controls::getStrafe,
            controls::getTurn,
            () -> headingOverride));

    controls
        .resetGyro()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Superstructure

    controls
        .shoot()
        .whileTrue(
            Commands.parallel(
                Commands.run(
                    () -> {
                      if (ZoneController.getInstance()
                                  .determineAction(ZoneController.getInstance().getCurrentZone())
                              == ZoneController.ZoneAction.FERRY_LEFT
                          || ZoneController.getInstance()
                                  .determineAction(ZoneController.getInstance().getCurrentZone())
                              == ZoneController.ZoneAction.FERRY_RIGHT) {}
                    })))
        .onFalse(
            Commands.runOnce(
                () -> {
                  headingOverride = Optional.empty();
                  drive.setGoal(Drive.Goal.TELEOP);
                }));

    controls
        .forceFerryLeft()
        .whileTrue(
            Commands.parallel(
                Commands.run(
                    () -> {
                      Translation2d robot =
                          RobotState.getInstance().getEstimatedPose().getTranslation();
                      Translation2d target =
                          AllianceFlipUtil.apply(ZoneController.FERRY_LEFT_TARGET);
                      headingOverride = Optional.of(target.minus(robot).getAngle());
                    })))
        .onFalse(
            Commands.runOnce(
                () -> {
                  headingOverride = Optional.empty();
                }));

    controls
        .forceFerryRight()
        .whileTrue(
            Commands.parallel(
                Commands.run(
                    () -> {
                      Translation2d robot =
                          RobotState.getInstance().getEstimatedPose().getTranslation();
                      Translation2d target =
                          AllianceFlipUtil.apply(ZoneController.FERRY_RIGHT_TARGET);
                      headingOverride = Optional.of(target.minus(robot).getAngle());
                    })))
        .onFalse(
            Commands.runOnce(
                () -> {
                  headingOverride = Optional.empty();
                }));
  }

  public void periodic() {
    // Sync dashboard shot mode
    ShotMode dashMode = shotModeChooser.get();
    if (dashMode != null) {
      RobotState.getInstance().setShotMode(dashMode);
    }

    zoneVisualizer.periodic();
    RobotState.getInstance().updateLogger();
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
