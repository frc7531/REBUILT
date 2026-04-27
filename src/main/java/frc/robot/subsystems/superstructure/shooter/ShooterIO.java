// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leftConnected = false;
    public double leftTempCelsius = 0.0;
    public double leftVelocityRPM = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTorqueCurrentAmps = 0.0;

    public boolean rightConnected = false;
    public double rightTempCelsius = 0.0;
    public double rightVelocityRPM = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTorqueCurrentAmps = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVelocity(double velocityRPM, double feedforward) {}

  default void stop() {}

  default void setPID(double kP, double kD, double kV, double kS, double kA) {}
}
