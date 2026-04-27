// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.floor.Floor;
import frc.robot.subsystems.superstructure.floor.FloorIO;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.hood.HoodIO;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShooterIO;
import frc.robot.subsystems.superstructure.transfer.Transfer;
import frc.robot.subsystems.superstructure.transfer.TransferIO;
import frc.robot.subsystems.superstructure.wall.Wall;
import frc.robot.subsystems.superstructure.wall.WallIO;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableBoolean;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.zonecontroller.ZoneController;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  public enum Goal {
    IDLE,
    SHOOT,
    INDEX,
    UNCLOG,
    UNDEX,
    FERRY,
    STOW
  }

  public enum SystemState {
    IDLE,
    PREPARING,
    READY,
    SHOOTING,
    INDEXING,
    UNCLOGGING,
    UNDEXING,
    FERRYING,
    STOWING
  }

  // Shot maps (distance -> value)
  private static final InterpolatingDoubleTreeMap SHOOTER_RPM_MAP =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap HOOD_DEG_MAP = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap WALL_RPM_MAP = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap TRANSFER_RPM_MAP =
      new InterpolatingDoubleTreeMap();

  static {
    SHOOTER_RPM_MAP.put(3.0, 2500.0);
    SHOOTER_RPM_MAP.put(3.895, 2700.0);
    SHOOTER_RPM_MAP.put(1.93, 2375.0);

    TRANSFER_RPM_MAP.put(3.0, 4000.0);
    TRANSFER_RPM_MAP.put(3.895, 4000.0);
    TRANSFER_RPM_MAP.put(1.93, 4000.0);

    HOOD_DEG_MAP.put(3.0, 7.5);
    HOOD_DEG_MAP.put(3.895, 13.0);
    HOOD_DEG_MAP.put(1.93, 2.0);

    WALL_RPM_MAP.put(3.0, 4000.0);
    WALL_RPM_MAP.put(3.895, 4000.0);
    WALL_RPM_MAP.put(1.93, 4000.0);
  }

  // Calculation mode: when on, overrides map lookups with manual values
  private static final LoggedTunableBoolean calculationMode =
      new LoggedTunableBoolean("Superstructure/CalculationMode", false);
  private static final LoggedTunableNumber calShooterRPM =
      new LoggedTunableNumber("Superstructure/Calculations/ShooterRPM", 0.0);
  private static final LoggedTunableNumber calHoodAngle =
      new LoggedTunableNumber("Superstructure/Calculations/HoodAngle", 0.0);
  private static final LoggedTunableNumber calWallRPM =
      new LoggedTunableNumber("Superstructure/Calculations/WallRPM", 0.0);
  private static final LoggedTunableNumber calTransferRPM =
      new LoggedTunableNumber("Superstructure/Calculations/TransferRPM", 0.0);

  // Other tunables
  private static final LoggedTunableNumber compensationRPM =
      new LoggedTunableNumber("Superstructure/CompensationRPM", 150);
  private static final LoggedTunableNumber unclogRPM =
      new LoggedTunableNumber("Superstructure/UnclogRPM", 150);
  private static final LoggedTunableNumber floorRPM =
      new LoggedTunableNumber("Superstructure/FloorRPM", 300);
  private static final LoggedTunableNumber floorTorque =
      new LoggedTunableNumber("Superstructure/FloorTorque", 10.0);
  private static final LoggedTunableNumber floorVolts =
      new LoggedTunableNumber("Superstructure/FloorVolts", 4.5);

  private static final LoggedTunableNumber ferryShooterRPM =
      new LoggedTunableNumber("Superstructure/Ferry/ShootRPM", 5000);
  private static final LoggedTunableNumber ferryHoodAngle =
      new LoggedTunableNumber("Superstructure/Ferry/HoodDeg", 30.0);
  private static final LoggedTunableNumber ferryWallRPM =
      new LoggedTunableNumber("Superstructure/Ferry/WallRPM", 2000.0);
  private static final LoggedTunableNumber ferryTransferRPM =
      new LoggedTunableNumber("Superstructure/Ferry/TransferRPM", 2000.0);

  // Components
  private final Shooter shooter;
  private final Hood hood;
  private final Wall wall;
  private final Transfer transfer;
  private final Floor floor;

  @Getter private Goal goal = Goal.IDLE;
  @Getter private SystemState state = SystemState.IDLE;

  public Superstructure(
      ShooterIO shooterIO, HoodIO hoodIO, WallIO wallIO, TransferIO transferIO, FloorIO floorIO) {
    shooter = new Shooter(shooterIO);
    hood = new Hood(hoodIO);
    wall = new Wall(wallIO);
    transfer = new Transfer(transferIO);
    floor = new Floor(floorIO);
  }

  @Override
  public void periodic() {
    shooter.periodic();
    hood.periodic();
    wall.periodic();
    transfer.periodic();
    floor.periodic();

    state = evaluateState();
    applyOutputs();

    Logger.recordOutput("Superstructure/Goal", goal.toString());
    Logger.recordOutput("Superstructure/State", state.toString());
    Logger.recordOutput("Superstructure/ShooterReady", shooter.isReady());
    Logger.recordOutput("Superstructure/HoodReady", hood.isReady());
    Logger.recordOutput("Superstructure/DistanceToTarget", getDistanceToTarget());
    Logger.recordOutput("Superstructure/CalculationMode", calculationMode.get());

    var params = getShotParams();
    Logger.recordOutput("Superstructure/Active/ShooterRPM", params.shooterRPM);
    Logger.recordOutput("Superstructure/Active/HoodDeg", params.hoodDeg);
    Logger.recordOutput("Superstructure/Active/WallRPM", params.wallRPM);
    Logger.recordOutput("Superstructure/Active/TransferRPM", params.transferRPM);

    RobotState.getInstance()
        .setSuperstructureActivity(
            switch (state) {
              case IDLE -> RobotState.SuperstructureActivity.IDLE;
              case PREPARING, READY -> RobotState.SuperstructureActivity.PREPARING;
              case SHOOTING -> RobotState.SuperstructureActivity.SHOOTING;
              case INDEXING -> RobotState.SuperstructureActivity.INDEXING;
              case UNCLOGGING -> RobotState.SuperstructureActivity.UNCLOGGING;
              case UNDEXING -> RobotState.SuperstructureActivity.UNDEXING;
              case FERRYING -> RobotState.SuperstructureActivity.FERRYING;
              case STOWING -> RobotState.SuperstructureActivity.STOWING;
            });

    LoggedTracer.record("Superstructure");
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public boolean isReady() {
    return state == SystemState.READY;
  }

  // Shot parameter resolution

  private record ActiveShotParams(
      double shooterRPM, double hoodDeg, double wallRPM, double transferRPM) {}

  private ActiveShotParams getShotParams() {
    if (calculationMode.get()) {
      return new ActiveShotParams(
          calShooterRPM.get(), calHoodAngle.get(), calWallRPM.get(), calTransferRPM.get());
    }
    double distance = getDistanceToTarget();
    return new ActiveShotParams(
        SHOOTER_RPM_MAP.get(distance),
        HOOD_DEG_MAP.get(distance),
        WALL_RPM_MAP.get(distance),
        TRANSFER_RPM_MAP.get(distance));
  }

  // State machine

  private SystemState evaluateState() {
    return switch (goal) {
      case IDLE -> SystemState.IDLE;
      case SHOOT -> {
        if (shooter.isReady() && hood.isReady()) yield SystemState.READY;
        yield SystemState.PREPARING;
      }
      case INDEX -> SystemState.INDEXING;
      case UNCLOG -> SystemState.UNCLOGGING;
      case UNDEX -> SystemState.UNDEXING;
      case FERRY -> SystemState.FERRYING;
      case STOW -> SystemState.STOWING;
    };
  }

  private void applyOutputs() {
    var params = getShotParams();
    boolean forceHoodStow = ZoneController.getInstance().shouldStowHood();

    switch (state) {
      case IDLE -> {
        shooter.stop();
        if (forceHoodStow) hood.stow();
        else hood.setAngle(params.hoodDeg);
        wall.stop();
        transfer.stop();
        floor.stop();
      }
      case PREPARING -> {
        shooter.runVelocity(params.shooterRPM);
        if (forceHoodStow) hood.stow();
        else hood.setAngle(params.hoodDeg);
        wall.runVelocity(params.wallRPM);
        transfer.stop();
        floor.runVolts(floorVolts.get());
      }
      case READY -> {
        shooter.runVelocity(params.shooterRPM);
        if (forceHoodStow) hood.stow();
        else hood.setAngle(params.hoodDeg);
        wall.runVelocity(params.wallRPM);
        transfer.runVelocity(params.transferRPM);
        floor.runVolts(floorVolts.get());
      }
      case SHOOTING -> {
        shooter.runVelocity(params.shooterRPM + compensationRPM.get());
        if (forceHoodStow) hood.stow();
        else hood.setAngle(params.hoodDeg);
        wall.runVelocity(params.wallRPM);
        transfer.runVelocity(params.transferRPM);
        floor.runVolts(floorVolts.get());
      }
      case INDEXING -> {
        shooter.stop();
        hood.stop();
        wall.stop();
        transfer.stop();
        floor.runVolts(floorVolts.get());
      }
      case UNCLOGGING -> {
        shooter.reverse(unclogRPM.get());
        hood.stow();
        wall.reverse(unclogRPM.get());
        transfer.reverse(unclogRPM.get());
        floor.reverse(unclogRPM.get());
      }
      case UNDEXING -> {
        floor.reverse(floorRPM.get());
      }
      case FERRYING -> {
        shooter.runVelocity(ferryShooterRPM.get());
        if (forceHoodStow) hood.stow();
        else hood.setAngle(ferryHoodAngle.get());
        wall.runVelocity(ferryWallRPM.get());
        transfer.runVelocity(ferryTransferRPM.get());
        floor.runVolts(floorVolts.get());
      }
      case STOWING -> {
        hood.stow();
      }
    }
  }

  private double getDistanceToTarget() {
    Translation2d target = ZoneController.getInstance().getAimTarget();
    return RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(target);
  }

  public void confirmShoot() {
    if (state == SystemState.READY) {
      state = SystemState.SHOOTING;
    }
  }

  // Commands

  public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE));
  }

  public Command indexCommand() {
    return startEnd(() -> setGoal(Goal.INDEX), () -> setGoal(Goal.IDLE));
  }

  public Command unclogCommand() {
    return startEnd(() -> setGoal(Goal.UNCLOG), () -> setGoal(Goal.IDLE));
  }

  public Command idleCommand() {
    return runOnce(() -> setGoal(Goal.IDLE));
  }

  public Command undexCommand() {
    return runOnce(() -> setGoal(Goal.UNDEX));
  }

  public Command ferryCommand() {
    return runOnce(() -> setGoal(Goal.FERRY));
  }

  public Command stowCommand() {
    return runOnce(() -> setGoal(Goal.STOW));
  }
}
