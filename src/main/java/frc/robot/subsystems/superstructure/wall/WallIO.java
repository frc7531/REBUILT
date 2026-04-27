// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.wall;

import org.littletonrobotics.junction.AutoLog;

public interface WallIO {
  @AutoLog
  public static class WallIOInputs {
    public boolean connected = false;
    public double tempCelsius = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
  }

  default void updateInputs(WallIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void runVelocity(double velocityRPM) {}

  default void stop() {}

  default void setPID(double kP, double kD, double kV, double kS) {}
}
