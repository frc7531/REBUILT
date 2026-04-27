// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

public class PracTunerConstants {

  public static final String kCANBusName = "Tim";
  public static final int kPigeonId = 10;

  public static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(63)
          .withKI(0)
          .withKD(1.2)
          .withKS(0.1)
          .withKV(3.1)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  public static final Slot0Configs driveGains =
      new Slot0Configs().withKP(0.3).withKI(0).withKD(0).withKS(0.04843).withKV(0.76140);

  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.69);
  public static final double kCoupleRatio = 0;
  public static final double kDriveGearRatio = 6.75;
  public static final double kSteerGearRatio = 25;
  public static final Distance kWheelRadius = Inches.of(1.928);
  public static final boolean kInvertLeftSide = false;
  public static final boolean kInvertRightSide = true;

  public static final double trackWidthX = Units.inchesToMeters(24.5);
  public static final double trackWidthY = Units.inchesToMeters(24.5);
  public static final double robotWidthY = Units.inchesToMeters(30) + 2 * Units.inchesToMeters(2.0);
  public static final double robotWidthX = Units.inchesToMeters(30) + 2 * Units.inchesToMeters(2.0);

  // Front Left
  public static final int kFrontLeftDriveMotorId = 43;
  public static final int kFrontLeftSteerMotorId = 42;
  public static final int kFrontLeftEncoderId = 41;
  public static final Angle kFrontLeftEncoderOffset = Rotations.of(0.40234375);
  public static final boolean kFrontLeftSteerMotorInverted = false;
  public static final boolean kFrontLeftEncoderInverted = false;
  public static final Distance kFrontLeftXPos = Inches.of(12.5);
  public static final Distance kFrontLeftYPos = Inches.of(12.5);

  // Front Right
  public static final int kFrontRightDriveMotorId = 33;
  public static final int kFrontRightSteerMotorId = 32;
  public static final int kFrontRightEncoderId = 31;
  public static final Angle kFrontRightEncoderOffset = Rotations.of(-0.011962890625);
  public static final boolean kFrontRightSteerMotorInverted = false;
  public static final boolean kFrontRightEncoderInverted = false;
  public static final Distance kFrontRightXPos = Inches.of(12.5);
  public static final Distance kFrontRightYPos = Inches.of(-12.5);

  // Back Left
  public static final int kBackLeftDriveMotorId = 13;
  public static final int kBackLeftSteerMotorId = 12;
  public static final int kBackLeftEncoderId = 11;
  public static final Angle kBackLeftEncoderOffset = Rotations.of(0.30908203125);
  public static final boolean kBackLeftSteerMotorInverted = false;
  public static final boolean kBackLeftEncoderInverted = false;
  public static final Distance kBackLeftXPos = Inches.of(-12.5);
  public static final Distance kBackLeftYPos = Inches.of(12.5);

  // Back Right
  public static final int kBackRightDriveMotorId = 23;
  public static final int kBackRightSteerMotorId = 22;
  public static final int kBackRightEncoderId = 21;
  public static final Angle kBackRightEncoderOffset = Rotations.of(0.17333984375);
  public static final boolean kBackRightSteerMotorInverted = false;
  public static final boolean kBackRightEncoderInverted = false;
  public static final Distance kBackRightXPos = Inches.of(-12.5);
  public static final Distance kBackRightYPos = Inches.of(-12.5);
}
