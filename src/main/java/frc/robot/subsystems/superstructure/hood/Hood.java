// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.RobotState;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Hood {

  public enum Goal {
    IDLE,
    STOW,
    AIM
  }

  public enum State {
    IDLE,
    STOWING,
    MOVING_TO_ANGLE,
    AT_ANGLE
  }

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", 450.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", 3.75);
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG", 0.5);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS", 3.0);

  private static final LoggedTunableNumber velocity = new LoggedTunableNumber("Hood/Velocity", 8.0);
  private static final LoggedTunableNumber acceleration =
      new LoggedTunableNumber("Hood/Acceleration", 8.0);
  private static final LoggedTunableNumber jerk = new LoggedTunableNumber("Hood/Jerk", 0.0);

  private static final LoggedTunableNumber minDeg = new LoggedTunableNumber("Hood/MinDeg", 0.0);
  private static final LoggedTunableNumber maxDeg = new LoggedTunableNumber("Hood/MaxDeg", 29.4);
  private static final LoggedTunableNumber stowDeg = new LoggedTunableNumber("Hood/StowDeg", 0.0);
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg", 2.2);
  private static final LoggedTunableNumber readyDebounceSec =
      new LoggedTunableNumber("Hood/ReadyDebounceSec", 0.08);

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  @Getter private Goal goal = Goal.IDLE;
  @Getter private State state = State.IDLE;
  @Getter private double targetDeg = 0.0;

  private final Debouncer readyDebouncer =
      new Debouncer(readyDebounceSec.get(), Debouncer.DebounceType.kFalling);

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kG.hasChanged(hashCode())
        || kV.hasChanged(hashCode())) {
      io.setPID(kP.get(), kD.get(), kG.get(), kV.get(), kS.get());
    }

    if (velocity.hasChanged(hashCode())
        || acceleration.hasChanged(hashCode())
        || jerk.hasChanged(hashCode())) {
      io.setMotionMagic(velocity.get(), acceleration.get(), jerk.get());
    }

    if (minDeg.hasChanged(hashCode()) || maxDeg.hasChanged(hashCode())) {
      io.setMinMax(minDeg.get(), maxDeg.get());
    }

    if (readyDebounceSec.hasChanged(hashCode())) {
      readyDebouncer.setDebounceTime(readyDebounceSec.get());
    }

    state = evaluateState();
    applyOutputs();

    RobotState.getInstance().setHoodAngleDeg(inputs.angleDegrees);
    RobotState.getInstance().setHoodCurrentDraw(inputs.supplyCurrentAmps);

    Logger.recordOutput("Hood/Goal", goal.toString());
    Logger.recordOutput("Hood/State", state.toString());
    Logger.recordOutput("Hood/TargetDeg", targetDeg);
    Logger.recordOutput("Hood/MeasuredDeg", inputs.angleDegrees);
    Logger.recordOutput("Hood/AtSetpoint", atSetpoint());
    Logger.recordOutput("Hood/Ready", isReady());
    LoggedTracer.record("Hood");
  }

  private State evaluateState() {
    return switch (goal) {
      case IDLE -> State.IDLE;
      case STOW -> atTarget(stowDeg.get()) ? State.AT_ANGLE : State.STOWING;
      case AIM -> atTarget(targetDeg) ? State.AT_ANGLE : State.MOVING_TO_ANGLE;
    };
  }

  private void applyOutputs() {
    if (state == State.IDLE) {
      io.stop();
      return;
    }
    double commanded =
        switch (goal) {
          case STOW -> stowDeg.get();
          case AIM -> targetDeg;
          case IDLE -> 0.0;
        };
    io.setAngle(MathUtil.clamp(commanded, minDeg.get(), maxDeg.get()));
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public void setAngle(double angleDegrees) {
    this.goal = Goal.AIM;
    this.targetDeg = angleDegrees;
  }

  public void stow() {
    this.goal = Goal.STOW;
  }

  public void stop() {
    this.goal = Goal.IDLE;
    this.targetDeg = 0.0;
    readyDebouncer.calculate(false);
  }

  private boolean atTarget(double target) {
    return Math.abs(inputs.angleDegrees - target) <= toleranceDeg.get();
  }

  public boolean atSetpoint() {
    return state == State.AT_ANGLE;
  }

  public boolean isReady() {
    return readyDebouncer.calculate(atSetpoint());
  }
}
