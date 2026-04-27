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
import frc.robot.util.LoggedTunableEnum;

public class CompTunerConstants {

  private enum ratio {
    R1(7.09),
    R2(6.03),
    R3(5.2734375);

    public final double driveRatio;

    ratio(double driveRatio) {
      this.driveRatio = driveRatio;
    }
  }

  private static final LoggedTunableEnum<ratio> driveRatio =
      new LoggedTunableEnum<>("TunerConstants/DriveRatio", ratio.class, ratio.R2);

  public static final String kCANBusName = "Tyrone";
  public static final int kPigeonId = 2;

  public static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(0.5)
          .withKS(0.1)
          .withKV(2.49)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  public static final Slot0Configs driveGains =
      new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0.0).withKV(0.76140);

  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.81584);
  public static final double kCoupleRatio = 0;
  public static final double kDriveGearRatio = driveRatio.getDouble(r -> r.driveRatio);
  public static final double kSteerGearRatio = 26;
  public static final Distance kWheelRadius = Inches.of(2);
  public static final boolean kInvertLeftSide = false;
  public static final boolean kInvertRightSide = true;

  public static final double trackWidthX = Units.inchesToMeters(21.25);
  public static final double trackWidthY = Units.inchesToMeters(21.25);
  public static final double robotWidthY = Units.inchesToMeters(27 + 3.5 + 3.5);
  public static final double robotWidthX = Units.inchesToMeters(26.5 + 3.5 + 3.0);

  // Front Left
  public static final int kFrontLeftDriveMotorId = 10;
  public static final int kFrontLeftSteerMotorId = 11;
  public static final int kFrontLeftEncoderId = 12;
  public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.1796875);
  public static final boolean kFrontLeftSteerMotorInverted = false;
  public static final boolean kFrontLeftEncoderInverted = false;
  public static final Distance kFrontLeftXPos = Inches.of(10.625);
  public static final Distance kFrontLeftYPos = Inches.of(10.625);

  // Front Right
  public static final int kFrontRightDriveMotorId = 20;
  public static final int kFrontRightSteerMotorId = 21;
  public static final int kFrontRightEncoderId = 22;
  public static final Angle kFrontRightEncoderOffset = Rotations.of(0.414306640625);
  public static final boolean kFrontRightSteerMotorInverted = false;
  public static final boolean kFrontRightEncoderInverted = false;
  public static final Distance kFrontRightXPos = Inches.of(10.625);
  public static final Distance kFrontRightYPos = Inches.of(-10.625);

  // Back Left
  public static final int kBackLeftDriveMotorId = 30;
  public static final int kBackLeftSteerMotorId = 31;
  public static final int kBackLeftEncoderId = 32;
  public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.39990234375);
  public static final boolean kBackLeftSteerMotorInverted = false;
  public static final boolean kBackLeftEncoderInverted = false;
  public static final Distance kBackLeftXPos = Inches.of(-10.625);
  public static final Distance kBackLeftYPos = Inches.of(10.625);

  // Back Right
  public static final int kBackRightDriveMotorId = 40;
  public static final int kBackRightSteerMotorId = 41;
  public static final int kBackRightEncoderId = 42;
  public static final Angle kBackRightEncoderOffset = Rotations.of(0.12255859375);
  public static final boolean kBackRightSteerMotorInverted = false;
  public static final boolean kBackRightEncoderInverted = false;
  public static final Distance kBackRightXPos = Inches.of(-10.625);
  public static final Distance kBackRightYPos = Inches.of(-10.625);
}
