// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.wall;

import frc.robot.RobotState;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Wall {

  public enum Goal {
    IDLE,
    VOLTAGE,
    VELOCITY,
    REVERSE
  }

  public enum State {
    IDLE,
    RUNNING_VOLTS,
    SPINNING_UP,
    AT_VELOCITY,
    REVERSING
  }

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Wall/kP", 2);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Wall/kD", 0.0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Wall/kV", 0.15);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Wall/kS", 19.0);
  private static final LoggedTunableNumber voltsTolerance =
      new LoggedTunableNumber("Wall/VoltsTolerance", 0.1);
  private static final LoggedTunableNumber rpmTolerance =
      new LoggedTunableNumber("Wall/RPMTolerance", 50);

  private WallIO io;
  private WallIOInputsAutoLogged inputs = new WallIOInputsAutoLogged();

  @Getter private Goal goal = Goal.IDLE;
  @Getter private State state = State.IDLE;
  @Getter private double targetVolts = 0.0;
  @Getter private double targetRPM = 0.0;

  public Wall(WallIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wall", inputs);

    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kS.hasChanged(hashCode())) {
      io.setPID(kP.get(), kD.get(), kV.get(), kS.get());
    }

    state = evaluateState();
    applyOutputs();

    RobotState.getInstance().setWallVelocityRPM(inputs.velocityRPM);
    RobotState.getInstance().setWallCurrentDraw(inputs.supplyCurrentAmps);

    Logger.recordOutput("Wall/Goal", goal.toString());
    Logger.recordOutput("Wall/State", state.toString());
    Logger.recordOutput("Wall/TargetRPM", targetRPM);
    Logger.recordOutput("Wall/MeasuredRPM", inputs.velocityRPM);
    Logger.recordOutput("Wall/AtSetpoint", atSetpoint());
    LoggedTracer.record("Wall");
  }

  private State evaluateState() {
    return switch (goal) {
      case IDLE -> State.IDLE;
      case VOLTAGE -> State.RUNNING_VOLTS;
      case VELOCITY -> {
        if (Math.abs(inputs.velocityRPM - targetRPM) <= rpmTolerance.get()) yield State.AT_VELOCITY;
        yield State.SPINNING_UP;
      }
      case REVERSE -> State.REVERSING;
    };
  }

  private void applyOutputs() {
    switch (state) {
      case IDLE -> io.stop();
      case RUNNING_VOLTS -> io.runVolts(targetVolts);
      case SPINNING_UP, AT_VELOCITY -> io.runVelocity(targetRPM);
      case REVERSING -> io.runVelocity(targetRPM);
    }
  }

  public void runVolts(double volts) {
    goal = Goal.VOLTAGE;
    targetVolts = volts;
  }

  public void runVelocity(double velocityRPM) {
    goal = Goal.VELOCITY;
    targetRPM = velocityRPM;
  }

  public void reverse(double rpm) {
    goal = Goal.REVERSE;
    targetRPM = -Math.abs(rpm);
  }

  public void stop() {
    goal = Goal.IDLE;
    targetRPM = 0.0;
  }

  public boolean atSetpoint() {
    return state == State.AT_VELOCITY;
  }
}
