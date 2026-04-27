// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.Rollers;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Rollers {

  public enum Goal {
    IDLE,
    DUTY,
    VOLTAGE,
    VELOCITY,
    REVERSE
  }

  public enum State {
    IDLE,
    RUNNING_DUTY,
    RUNNING_VOLTS,
    SPINNING_UP,
    AT_VELOCITY,
    REVERSING
  }

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Rollers/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Rollers/kD");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Rollers/kV");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Rollers/kS");
  private static final LoggedTunableNumber rpmTolerance =
      new LoggedTunableNumber("Rollers/RPMTolerance");

  static {
    switch (Constants.getRobot()) {
      case COMPBOT, PRACBOT -> {
        kP.initDefault(10.0);
        kD.initDefault(0.0);
        kV.initDefault(0);
        kS.initDefault(0);

        rpmTolerance.initDefault(50);
      }
      case SIMBOT -> {
        kP.initDefault(0.0);
        kD.initDefault(0.0);
        kV.initDefault(0);
        kS.initDefault(0);

        rpmTolerance.initDefault(50);
      }
    }
  }

  private final RollersIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  @Getter private Goal goal = Goal.IDLE;
  @Getter private State state = State.IDLE;
  @Getter private double targetVolts = 0.0;
  @Getter private double targetRPM = 0.0;
  @Getter private double targetDuty = 0.0;

  public Rollers(RollersIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);

    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kS.hasChanged(hashCode())) {
      io.setPID(kP.get(), kD.get(), kV.get(), kS.get());
    }

    state = evaluateState();
    applyOutputs();

    RobotState.getInstance().setRollersVelocityRPM(inputs.velocityRPM);
    RobotState.getInstance().setRollersCurrentDraw(inputs.supplyCurrentAmps);

    Logger.recordOutput("Rollers/Goal", goal.toString());
    Logger.recordOutput("Rollers/State", state.toString());
    Logger.recordOutput("Rollers/TargetRPM", targetRPM);
    Logger.recordOutput("Rollers/MeasuredRPM", inputs.velocityRPM);
    Logger.recordOutput("Rollers/AtVelocity", atVelocity());

    LoggedTracer.record("Rollers");
  }

  private State evaluateState() {
    return switch (goal) {
      case IDLE -> State.IDLE;
      case DUTY -> State.RUNNING_DUTY;
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
      case RUNNING_DUTY -> io.runDutyCycle(targetDuty);
      case RUNNING_VOLTS -> io.runVolts(targetVolts);
      case SPINNING_UP, AT_VELOCITY -> io.runVelocity(targetRPM);
      case REVERSING -> io.runVelocity(-Math.abs(targetRPM));
    }
  }

  public void runVolts(double volts) {
    goal = Goal.VOLTAGE;
    targetVolts = volts;
  }

  public void runDutyCycle(double output) {
    goal = Goal.DUTY;
    targetDuty = output;
  }

  public void runVelocity(double rpm) {
    goal = Goal.VELOCITY;
    targetRPM = rpm;
  }

  public void reverse(double rpm) {
    goal = Goal.REVERSE;
    targetRPM = rpm;
  }

  public void stop() {
    goal = Goal.IDLE;
    targetVolts = 0.0;
    targetRPM = 0.0;
    targetDuty = 0.0;
  }

  public boolean atVelocity() {
    return state == State.AT_VELOCITY;
  }
}
