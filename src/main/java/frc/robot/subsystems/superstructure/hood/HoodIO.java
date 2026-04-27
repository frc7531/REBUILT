// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean connected = false;
    public double tempCelsius = 0.0;
    public double angleDegrees = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void setAngle(double angleDegrees) {}

  default void stop() {}

  default void setPID(double kP, double kD, double kG, double kV, double kS) {}

  default void setMotionMagic(double cruiseVelo, double acceleration, double jerk) {}

  default void setMinMax(double minDeg, double maxDeg) {}
}
