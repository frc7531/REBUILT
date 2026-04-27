// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.RobotState;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Shooter {

  public enum Goal {
    IDLE,
    VELOCITY,
    REVERSE
  }

  public enum State {
    IDLE,
    SPINNING_UP,
    AT_VELOCITY,
    REVERSING
  }

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 4);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0.0);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.23);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", 7.5);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/kA", 0.0);
  private static final LoggedTunableNumber ff = new LoggedTunableNumber("Shooter/FF", 0.0);
  private static final LoggedTunableNumber rpmTolerance =
      new LoggedTunableNumber("Shooter/RPMTolerance", 300.0);
  private static final LoggedTunableNumber readyDebounceSec =
      new LoggedTunableNumber("Shooter/ReadyDebounceSec", 0.10);
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  @Getter private Goal goal = Goal.IDLE;
  @Getter private State state = State.IDLE;
  @Getter private double targetRPM = 0.0;

  private final Debouncer readyDebouncer =
      new Debouncer(readyDebounceSec.get(), Debouncer.DebounceType.kFalling);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      io.setPID(kP.get(), kD.get(), kV.get(), kS.get(), kA.get());
    }

    if (readyDebounceSec.hasChanged(hashCode())) {
      readyDebouncer.setDebounceTime(readyDebounceSec.get());
    }

    state = evaluateState();
    applyOutputs();

    double avgRPM = (Math.abs(inputs.leftVelocityRPM) + Math.abs(inputs.rightVelocityRPM)) / 2.0;
    RobotState.getInstance().setShooterVelocityRPM(avgRPM);
    RobotState.getInstance()
        .setShooterCurrentDraw(inputs.leftSupplyCurrentAmps + inputs.rightSupplyCurrentAmps);

    Logger.recordOutput("Shooter/Goal", goal.toString());
    Logger.recordOutput("Shooter/State", state.toString());
    Logger.recordOutput("Shooter/TargetRPM", targetRPM);
    Logger.recordOutput("Shooter/MeasuredRPM", avgRPM);
    Logger.recordOutput("Shooter/AtSetpoint", atSetpoint());
    Logger.recordOutput("Shooter/Readu", isReady());
    LoggedTracer.record("Shooter");
  }

  private State evaluateState() {
    return switch (goal) {
      case IDLE -> State.IDLE;
      case VELOCITY -> {
        double avgRPM =
            (Math.abs(inputs.leftVelocityRPM) + Math.abs(inputs.rightVelocityRPM)) / 2.0;
        if (Math.abs(avgRPM - targetRPM) <= rpmTolerance.get()) yield State.AT_VELOCITY;
        yield State.SPINNING_UP;
      }
      case REVERSE -> State.REVERSING;
    };
  }

  private void applyOutputs() {
    switch (state) {
      case IDLE -> io.stop();
      case SPINNING_UP, AT_VELOCITY -> io.runVelocity(targetRPM, ff.get());
      case REVERSING -> io.runVelocity(targetRPM, 0);
    }
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
    readyDebouncer.calculate(false);
  }

  public boolean atSetpoint() {
    return state == State.AT_VELOCITY;
  }

  public boolean isReady() {
    return readyDebouncer.calculate(atSetpoint());
  }
}
