// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.floor;

import frc.robot.RobotState;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Floor {

  public enum Goal {
    IDLE,
    VOLTAGE,
    VELOCITY,
    REVERSE,
    TORQUE
  }

  public enum State {
    IDLE,
    RUNNING_VOLTS,
    SPINNING_UP,
    AT_VELOCITY,
    REVERSING,
    AT_OUTPUT
  }

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Floor/kP", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Floor/kD", 0.0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Floor/kV", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Floor/kS", 0.0);
  private static final LoggedTunableNumber rpmTolerance =
      new LoggedTunableNumber("Floor/RPMTolerance", 50.0);
  private static final LoggedTunableNumber outputTolerance =
      new LoggedTunableNumber("Floor/OutputTolerance", 1.0);

  private FloorIO io;
  private FloorIOInputsAutoLogged inputs = new FloorIOInputsAutoLogged();

  @Getter private Goal goal = Goal.IDLE;
  @Getter private State state = State.IDLE;
  @Getter private double targetVolts = 0.0;
  @Getter private double targetRPM = 0.0;
  @Getter private double targetOutput = 0.0;

  public Floor(FloorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Floor", inputs);

    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kS.hasChanged(hashCode())) {
      io.setPID(kP.get(), kD.get(), kV.get(), kS.get());
    }

    state = evaluateState();
    applyOutputs();

    RobotState.getInstance().setFloorVelocityRPM(inputs.velocityRPM);
    RobotState.getInstance().setFloorCurrentDraw(inputs.supplyCurrentAmps);

    Logger.recordOutput("Floor/Goal", goal.toString());
    Logger.recordOutput("Floor/State", state.toString());
    Logger.recordOutput("Floor/TargetVolts", targetVolts);
    Logger.recordOutput("Floor/TargetRPM", targetRPM);
    Logger.recordOutput("Floor/MeasuredRPM", inputs.velocityRPM);
    Logger.recordOutput("Floor/AtSetpoint", atSetpoint());
    LoggedTracer.record("Floor");
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
      case TORQUE -> {
        if ((inputs.torqueCurrentAmps - targetOutput) <= outputTolerance.get())
          yield State.AT_OUTPUT;
        yield State.SPINNING_UP;
      }
    };
  }

  private void applyOutputs() {
    switch (state) {
      case IDLE -> io.stop();
      case RUNNING_VOLTS -> io.runVolts(targetVolts);
      case SPINNING_UP, AT_VELOCITY -> io.runVelocity(targetRPM);
      case REVERSING -> io.runVelocity(targetRPM);
      case AT_OUTPUT -> io.runTorque(targetOutput);
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

  public void runTorque(double current) {
    goal = Goal.TORQUE;
    targetOutput = current;
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
