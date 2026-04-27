// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Pivot.Pivot;
import frc.robot.subsystems.intake.Pivot.PivotIO;
import frc.robot.subsystems.intake.Rollers.Rollers;
import frc.robot.subsystems.intake.Rollers.RollersIO;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public enum Goal {
    IDLE,
    STOW,
    DEPLOY,
    DEPLOY_HALF,
    INTAKE,
    EJECT,
    FEED
  }

  public enum State {
    IDLE,
    STOWING,
    STOWED,
    DEPLOYING,
    DEPLOYED,
    DEPLOYED_HALF,
    INTAKING,
    EJECTING,
    FEEDING
  }

  private static final LoggedTunableNumber stowedPosition =
      new LoggedTunableNumber("Intake/Pivot/StowedPosition", 0.0);
  private static final LoggedTunableNumber deployedPosition =
      new LoggedTunableNumber("Intake/Pivot/DeployedPosition", 15.0);
  private static final LoggedTunableNumber halfPosition =
      new LoggedTunableNumber("Intake/Pivot/HalfPosition", 5.0);
  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("Intake/Rollers/RunVolts", 9.0);
  private static final LoggedTunableNumber ejectVolts =
      new LoggedTunableNumber("Intake/Rollers/EjectVolts", -6.0);
  private static final LoggedTunableNumber slowRiseVolts =
      new LoggedTunableNumber("Intake/SlowRiseVolts", -1.75);
  private static final LoggedTunableNumber slowRollersVolts =
      new LoggedTunableNumber("Intake/SlowRollerVolts", 3.0);

  private final Pivot pivot;
  private final Rollers rollers;

  @Getter private Goal goal = Goal.IDLE;
  @Getter private State state = State.IDLE;

  public Intake(PivotIO pivotIO, RollersIO rollersIO) {
    pivot = new Pivot(pivotIO);
    rollers = new Rollers(rollersIO);
  }

  @Override
  public void periodic() {
    pivot.periodic();
    rollers.periodic();

    state = evaluateState();
    applyOutputs();

    RobotState.getInstance()
        .setIntakeActivity(
            switch (state) {
              case IDLE, STOWED -> RobotState.IntakeActivity.IDLE;
              case STOWING -> RobotState.IntakeActivity.STOWING;
              case DEPLOYING, DEPLOYED, DEPLOYED_HALF -> RobotState.IntakeActivity.DEPLOYING;
              case INTAKING -> RobotState.IntakeActivity.INTAKING;
              case EJECTING -> RobotState.IntakeActivity.IDLE;
              case FEEDING -> RobotState.IntakeActivity.FEEDING;
            });

    Logger.recordOutput("Intake/Goal", goal.toString());
    Logger.recordOutput("Intake/State", state.toString());

    LoggedTracer.record("Intake");
  }

  private State evaluateState() {
    return switch (goal) {
      case IDLE -> State.IDLE;
      case STOW -> pivot.atSetpoint() ? State.STOWED : State.STOWING;
      case DEPLOY -> pivot.atSetpoint() ? State.DEPLOYED : State.DEPLOYING;
      case DEPLOY_HALF -> pivot.atSetpoint() ? State.DEPLOYED_HALF : State.DEPLOYING;
      case INTAKE -> State.INTAKING;
      case EJECT -> State.EJECTING;
      case FEED -> State.FEEDING;
    };
  }

  private void applyOutputs() {
    switch (state) {
      case IDLE -> {
        pivot.stop();
        rollers.stop();
      }
      case STOWING -> {
        pivot.setPosition(stowedPosition.get());
        rollers.stop();
      }
      case STOWED -> {
        pivot.setPosition(stowedPosition.get());
        rollers.stop();
      }
      case DEPLOYING -> {
        double target = goal == Goal.DEPLOY_HALF ? halfPosition.get() : deployedPosition.get();
        pivot.setPosition(target);
        rollers.stop();
      }
      case DEPLOYED -> {
        pivot.setPosition(deployedPosition.get());
        rollers.stop();
      }
      case DEPLOYED_HALF -> {
        pivot.setPosition(halfPosition.get());
        rollers.stop();
      }
      case INTAKING -> {
        // pivot.setPosition(deployedPosition.get());
        rollers.runVolts(rollerVolts.get());
      }
      case EJECTING -> {
        pivot.setPosition(deployedPosition.get());
        rollers.runVolts(ejectVolts.get());
      }
      case FEEDING -> {
        pivot.runVolts(slowRiseVolts.get());
        rollers.runVolts(slowRollersVolts.get());
      }
    }
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  // Commands

  public Command intakeCommand() {
    return startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.DEPLOY_HALF));
  }

  public Command stowCommand() {
    return runOnce(() -> setGoal(Goal.STOW));
  }

  public Command deployCommand() {
    return runOnce(() -> setGoal(Goal.DEPLOY));
  }

  public Command deployHalfCommand() {
    return runOnce(() -> setGoal(Goal.DEPLOY_HALF));
  }

  public Command ejectCommand() {
    return startEnd(() -> setGoal(Goal.EJECT), () -> setGoal(Goal.IDLE));
  }

  public Command idleCommand() {
    return runOnce(() -> setGoal(Goal.IDLE));
  }

  public Command feedCommand() {
    return runOnce(() -> setGoal(Goal.FEED));
  }
}
