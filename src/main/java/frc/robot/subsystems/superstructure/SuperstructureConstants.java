// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.CANBus;

public class SuperstructureConstants {
  public static final CANBus canbus = CANBus.roboRIO();

  public static final int leftShooterId = 50;
  public static final int rightShooterId = 51;
  public static final double shooterReduction = 2.0;
  public static final double shooterStatorCurrentLimit = 100.0;
  public static final double shooterSupplyCurrentLimit = 60.0;

  public static final int hoodMotorId = 52;
  public static final int tbId = 53;
  public static final boolean tbInverted = true;
  public static final double hoodReduction = (360.0) / (20.0);
  public static final double hoodStatorCurrentLimit = 80.0;
  public static final double hoodSupplyCurrentLimit = 30.0;

  public static final int rollerWallMotorId = 54;
  public static final double rollerWallStatorCurrentLimit = 30.0;
  public static final double rollerWallSupplyCurrentLimit = 20.0;
  public static final double rollerWallReduction = 1.0;

  public static final int transferMotorId = 55;
  public static final double transferStatorCurrentLimit = 30.0;
  public static final double transferSupplyCurrentLimit = 20.0;
  public static final double transferReduction = 1.0;

  public static final int floorMotorId = 56;
  public static final double floorStatorCurrentLimit = 80.0;
  public static final double floorSupplyCurrentLimit = 20.0;
  public static final double floorReduction = 1.0;
}
