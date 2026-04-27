// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public class IntakeConstants {
  public static final CANBus canbus = CANBus.roboRIO();

  public static final int pivotMotorId = 60;

  public static final double pivotStatorCurrentLimit = 80;
  public static final int pivotSupplyCurrentLimit = 50;
  public static final double pivotReduction = 45.0;
  public static final int tbId = 62;
  public static final boolean tbInverted = true;
  public static final double tbReduction = 1.0;

  public static final int rollerMotorId = 61;
  public static final int rollerStatorCurrentLimit = 55;
  public static final int rollerSupplyCurrentLimit = 20;
  public static final double rollerReduction = 1.0;
}
