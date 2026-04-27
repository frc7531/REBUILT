// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.Pivot;

import frc.robot.RobotState;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableBoolean;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Pivot {

  public enum Goal {
    IDLE,
    VOLTAGE,
    POSITION,
  }

  public enum State {
    IDLE,
    RUNNING_VOLTS,
    HOMING_POSITION,
    AT_POSITION
  }

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/kP", 350.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/kD", 0.1);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/kG", 2.0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/kV", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/kS", 30.0);

  private static final LoggedTunableNumber velocity =
      new LoggedTunableNumber("Pivot/Velocity", 9.0);
  private static final LoggedTunableNumber acceleration =
      new LoggedTunableNumber("Pivot/Acceleration", 8.0);
  private static final LoggedTunableNumber jerk = new LoggedTunableNumber("Pivot/Jerk", 0.0);

  private static final LoggedTunableNumber min = new LoggedTunableNumber("Pivot/Min", 0.0);
  private static final LoggedTunableNumber max = new LoggedTunableNumber("Pivot/Max", 120.0);

  private static final LoggedTunableBoolean brakeMode =
      new LoggedTunableBoolean("Pivot/Brake", true);

  private static final LoggedTunableNumber degTolerance =
      new LoggedTunableNumber("Pivot/DegreeTolerance", 4);

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  @Getter private Goal goal = Goal.IDLE;
  @Getter private State state = State.IDLE;
  @Getter private double targetVolts = 0.0;
  @Getter private double targetDeg = 0.0;

  public Pivot(PivotIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kS.hasChanged(hashCode())) {
      io.setPID(kP.get(), kD.get(), kG.get(), kV.get(), kS.get());
      System.out.println("**Pivot PID changing**");
    }

    if (velocity.hasChanged(hashCode())
        || acceleration.hasChanged(hashCode())
        || jerk.hasChanged(hashCode())) {
      io.setMotionMagic(velocity.get(), acceleration.get(), jerk.get());
    }

    if (min.hasChanged(hashCode()) || max.hasChanged(hashCode())) {
      io.setMinMax(min.get(), max.get());
    }

    if (brakeMode.hasChanged(hashCode())) {
      io.setBrakeMode(brakeMode.get());
    }

    state = evaluateState();
    applyOutputs();

    RobotState.getInstance().setPivotDeg(inputs.angleDegrees);
    RobotState.getInstance().setPivotCurrentDraw(inputs.supplyCurrentAmps);

    Logger.recordOutput("Pivot/Goal", goal.toString());
    Logger.recordOutput("Pivot/State", state.toString());
    Logger.recordOutput("Pivot/TargetDeg", targetDeg);
    Logger.recordOutput("Pivot/MeasuredDeg", inputs.angleDegrees);
    Logger.recordOutput("Pivot/AtSetpoint", atSetpoint());

    LoggedTracer.record("Pivot");
  }

  private State evaluateState() {
    return switch (goal) {
      case IDLE -> State.IDLE;
      case VOLTAGE -> State.RUNNING_VOLTS;
      case POSITION -> {
        if (Math.abs(inputs.angleDegrees - targetDeg) <= degTolerance.get())
          yield State.AT_POSITION;
        yield State.HOMING_POSITION;
      }
    };
  }

  private void applyOutputs() {
    switch (state) {
      case IDLE -> io.stop();
      case RUNNING_VOLTS -> io.runVolts(targetVolts);
      case HOMING_POSITION, AT_POSITION -> io.setAngle(targetDeg);
    }
  }

  public void runVolts(double volts) {
    goal = Goal.VOLTAGE;
    targetVolts = volts;
  }

  public void setPosition(double positionDeg) {
    goal = Goal.POSITION;
    targetDeg = positionDeg;
  }

  public void stop() {
    goal = Goal.IDLE;
    targetVolts = 0.0;
    targetDeg = 0.0;
  }

  public boolean atSetpoint() {
    return state == State.AT_POSITION;
  }
}
