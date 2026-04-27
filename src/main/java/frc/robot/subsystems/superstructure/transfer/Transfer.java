// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.transfer;

import frc.robot.RobotState;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Transfer {

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

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Transfer/kP", 1.2);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Transfer/kD", 0.123);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Transfer/kV", 0.35);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Transfer/kS", 5.0);
  private static final LoggedTunableNumber voltsTolerance =
      new LoggedTunableNumber("Transfer/VoltsTolerance", 0.1);
  private static final LoggedTunableNumber rpmTolerance =
      new LoggedTunableNumber("Transfer/RPMTolerance", 50.0);

  private TransferIO io;
  private TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();

  @Getter private Goal goal = Goal.IDLE;
  @Getter private State state = State.IDLE;
  @Getter private double targetVolts = 0.0;
  @Getter private double targetRPM = 0.0;

  public Transfer(TransferIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Transfer", inputs);

    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kS.hasChanged(hashCode())) {
      io.setPID(kP.get(), kD.get(), kV.get(), kS.get());
    }

    state = evaluateState();
    applyOutputs();

    RobotState.getInstance().setTransferVelocityRPM(inputs.velocityRPM);
    RobotState.getInstance().setTransferCurrentDraw(inputs.supplyCurrentAmps);

    Logger.recordOutput("Transfer/Goal", goal.toString());
    Logger.recordOutput("Transfer/State", state.toString());
    Logger.recordOutput("Transfer/TargetRPM", targetRPM);
    Logger.recordOutput("Transfer/MeasuredRPM", inputs.velocityRPM);
    Logger.recordOutput("Transfer/AtSetpoint", atSetpoint());
    LoggedTracer.record("Transfer");
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
