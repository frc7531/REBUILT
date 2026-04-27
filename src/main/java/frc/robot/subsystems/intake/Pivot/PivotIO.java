// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public boolean connected = false;
    public double tempCelsius = 0.0;
    public double angleDegrees = 0.0;
    public double tbAngleDegrees = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void setAngle(double angleDegrees) {}

  default void setBrakeMode(boolean enabled) {}

  default void setPID(double kP, double kD, double kG, double kV, double kS) {}

  default void setMotionMagic(double cruiseVelo, double acceleration, double jerk) {}

  default void setMinMax(double minDeg, double maxDeg) {}

  default void stop() {}
}
