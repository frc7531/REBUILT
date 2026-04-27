// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

public class TunerConstants {
  // Drive train configuration
  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final DriveMotorArrangement kDriveMotorType =
      DriveMotorArrangement.TalonFX_Integrated;
  private static final SteerMotorArrangement kSteerMotorType =
      SteerMotorArrangement.TalonFX_Integrated;
  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;
  private static final Current kSlipCurrent = Amps.of(100.0);

  // Motor configs
  private static final TalonFXConfiguration driveInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(100.0))
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(50))
                  .withSupplyCurrentLimitEnable(true));
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(20))
                  .withSupplyCurrentLimitEnable(true));
  private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
  private static final Pigeon2Configuration pigeonConfigs = null;

  // Sim constants
  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
  private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  // Robot-specific selections
  private static final String kCANBusName;
  private static final int kPigeonId;
  private static final Slot0Configs steerGains;
  private static final Slot0Configs driveGains;

  // Physics constants
  public static final LinearVelocity kSpeedAt12Volts;
  private static final double kCoupleRatio;
  private static final double kDriveGearRatio;
  private static final double kSteerGearRatio;
  private static final Distance kWheelRadius;
  private static final boolean kInvertLeftSide;
  private static final boolean kInvertRightSide;

  // Dimensions
  public static final double trackWidthX;
  public static final double trackWidthY;
  public static final double robotWidthY;
  public static final double robotWidthX;

  // Module raw constants
  private static final int[] driveIds;
  private static final int[] steerIds;
  private static final int[] encoderIds;
  private static final Angle[] encoderOffsets;
  private static final boolean[] steerInverted;
  private static final boolean[] encoderInverted;
  private static final Distance[] xPos;
  private static final Distance[] yPos;

  static {
    // Select configuration based on robot type
    switch (Constants.robot) {
      case COMPBOT, SIMBOT:
        kCANBusName = CompTunerConstants.kCANBusName;
        kPigeonId = CompTunerConstants.kPigeonId;
        steerGains = CompTunerConstants.steerGains;
        driveGains = CompTunerConstants.driveGains;

        // Physics
        kSpeedAt12Volts = CompTunerConstants.kSpeedAt12Volts;
        kCoupleRatio = CompTunerConstants.kCoupleRatio;
        kDriveGearRatio = CompTunerConstants.kDriveGearRatio;
        kSteerGearRatio = CompTunerConstants.kSteerGearRatio;
        kWheelRadius = CompTunerConstants.kWheelRadius;
        kInvertLeftSide = CompTunerConstants.kInvertLeftSide;
        kInvertRightSide = CompTunerConstants.kInvertRightSide;

        // Dimensions
        trackWidthX = CompTunerConstants.trackWidthX;
        trackWidthY = CompTunerConstants.trackWidthY;
        robotWidthY = CompTunerConstants.robotWidthY;
        robotWidthX = CompTunerConstants.robotWidthX;

        // Module IDs
        driveIds =
            new int[] {
              CompTunerConstants.kFrontLeftDriveMotorId,
              CompTunerConstants.kFrontRightDriveMotorId,
              CompTunerConstants.kBackLeftDriveMotorId,
              CompTunerConstants.kBackRightDriveMotorId
            };
        steerIds =
            new int[] {
              CompTunerConstants.kFrontLeftSteerMotorId,
              CompTunerConstants.kFrontRightSteerMotorId,
              CompTunerConstants.kBackLeftSteerMotorId,
              CompTunerConstants.kBackRightSteerMotorId
            };
        encoderIds =
            new int[] {
              CompTunerConstants.kFrontLeftEncoderId,
              CompTunerConstants.kFrontRightEncoderId,
              CompTunerConstants.kBackLeftEncoderId,
              CompTunerConstants.kBackRightEncoderId
            };
        encoderOffsets =
            new Angle[] {
              CompTunerConstants.kFrontLeftEncoderOffset,
              CompTunerConstants.kFrontRightEncoderOffset,
              CompTunerConstants.kBackLeftEncoderOffset,
              CompTunerConstants.kBackRightEncoderOffset
            };
        steerInverted =
            new boolean[] {
              CompTunerConstants.kFrontLeftSteerMotorInverted,
              CompTunerConstants.kFrontRightSteerMotorInverted,
              CompTunerConstants.kBackLeftSteerMotorInverted,
              CompTunerConstants.kBackRightSteerMotorInverted
            };
        encoderInverted =
            new boolean[] {
              CompTunerConstants.kFrontLeftEncoderInverted,
              CompTunerConstants.kFrontRightEncoderInverted,
              CompTunerConstants.kBackLeftEncoderInverted,
              CompTunerConstants.kBackRightEncoderInverted
            };
        xPos =
            new Distance[] {
              CompTunerConstants.kFrontLeftXPos,
              CompTunerConstants.kFrontRightXPos,
              CompTunerConstants.kBackLeftXPos,
              CompTunerConstants.kBackRightXPos
            };
        yPos =
            new Distance[] {
              CompTunerConstants.kFrontLeftYPos,
              CompTunerConstants.kFrontRightYPos,
              CompTunerConstants.kBackLeftYPos,
              CompTunerConstants.kBackRightYPos
            };
        break;

      case PRACBOT:
        kCANBusName = PracTunerConstants.kCANBusName;
        kPigeonId = PracTunerConstants.kPigeonId;
        steerGains = PracTunerConstants.steerGains;
        driveGains = PracTunerConstants.driveGains;

        // Physics
        kSpeedAt12Volts = PracTunerConstants.kSpeedAt12Volts;
        kCoupleRatio = PracTunerConstants.kCoupleRatio;
        kDriveGearRatio = PracTunerConstants.kDriveGearRatio;
        kSteerGearRatio = PracTunerConstants.kSteerGearRatio;
        kWheelRadius = PracTunerConstants.kWheelRadius;
        kInvertLeftSide = PracTunerConstants.kInvertLeftSide;
        kInvertRightSide = PracTunerConstants.kInvertRightSide;

        // Dimensions
        trackWidthX = PracTunerConstants.trackWidthX;
        trackWidthY = PracTunerConstants.trackWidthY;
        robotWidthY = PracTunerConstants.robotWidthY;
        robotWidthX = PracTunerConstants.robotWidthX;

        // Module IDs
        driveIds =
            new int[] {
              PracTunerConstants.kFrontLeftDriveMotorId,
              PracTunerConstants.kFrontRightDriveMotorId,
              PracTunerConstants.kBackLeftDriveMotorId,
              PracTunerConstants.kBackRightDriveMotorId
            };
        steerIds =
            new int[] {
              PracTunerConstants.kFrontLeftSteerMotorId,
              PracTunerConstants.kFrontRightSteerMotorId,
              PracTunerConstants.kBackLeftSteerMotorId,
              PracTunerConstants.kBackRightSteerMotorId
            };
        encoderIds =
            new int[] {
              PracTunerConstants.kFrontLeftEncoderId,
              PracTunerConstants.kFrontRightEncoderId,
              PracTunerConstants.kBackLeftEncoderId,
              PracTunerConstants.kBackRightEncoderId
            };
        encoderOffsets =
            new Angle[] {
              PracTunerConstants.kFrontLeftEncoderOffset,
              PracTunerConstants.kFrontRightEncoderOffset,
              PracTunerConstants.kBackLeftEncoderOffset,
              PracTunerConstants.kBackRightEncoderOffset
            };
        steerInverted =
            new boolean[] {
              PracTunerConstants.kFrontLeftSteerMotorInverted,
              PracTunerConstants.kFrontRightSteerMotorInverted,
              PracTunerConstants.kBackLeftSteerMotorInverted,
              PracTunerConstants.kBackRightSteerMotorInverted
            };
        encoderInverted =
            new boolean[] {
              PracTunerConstants.kFrontLeftEncoderInverted,
              PracTunerConstants.kFrontRightEncoderInverted,
              PracTunerConstants.kBackLeftEncoderInverted,
              PracTunerConstants.kBackRightEncoderInverted
            };
        xPos =
            new Distance[] {
              PracTunerConstants.kFrontLeftXPos,
              PracTunerConstants.kFrontRightXPos,
              PracTunerConstants.kBackLeftXPos,
              PracTunerConstants.kBackRightXPos
            };
        yPos =
            new Distance[] {
              PracTunerConstants.kFrontLeftYPos,
              PracTunerConstants.kFrontRightYPos,
              PracTunerConstants.kBackLeftYPos,
              PracTunerConstants.kBackRightYPos
            };
        break;
      default:
        throw new IllegalStateException("Unknown robot type: " + Constants.robot);
    }
  }

  public static final CANBus kCANBus = new CANBus(kCANBusName, "./logs/example.hoot");

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  // Factory built using selected gains and physics
  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withCouplingGearRatio(kCoupleRatio)
              .withWheelRadius(kWheelRadius)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
              .withSlipCurrent(kSlipCurrent)
              .withSpeedAt12Volts(kSpeedAt12Volts)
              .withDriveMotorType(kDriveMotorType)
              .withSteerMotorType(kSteerMotorType)
              .withFeedbackSource(kSteerFeedbackType)
              .withDriveMotorInitialConfigs(driveInitialConfigs)
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withEncoderInitialConfigs(encoderInitialConfigs)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage);

  // Module constants populated based on robot type
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight;

  static {
    boolean[] driveInverted =
        new boolean[] {kInvertLeftSide, kInvertRightSide, kInvertLeftSide, kInvertRightSide};

    FrontLeft =
        ConstantCreator.createModuleConstants(
            steerIds[0],
            driveIds[0],
            encoderIds[0],
            encoderOffsets[0],
            xPos[0],
            yPos[0],
            driveInverted[0],
            steerInverted[0],
            encoderInverted[0]);

    FrontRight =
        ConstantCreator.createModuleConstants(
            steerIds[1],
            driveIds[1],
            encoderIds[1],
            encoderOffsets[1],
            xPos[1],
            yPos[1],
            driveInverted[1],
            steerInverted[1],
            encoderInverted[1]);

    BackLeft =
        ConstantCreator.createModuleConstants(
            steerIds[2],
            driveIds[2],
            encoderIds[2],
            encoderOffsets[2],
            xPos[2],
            yPos[2],
            driveInverted[2],
            steerInverted[2],
            encoderInverted[2]);

    BackRight =
        ConstantCreator.createModuleConstants(
            steerIds[3],
            driveIds[3],
            encoderIds[3],
            encoderOffsets[3],
            xPos[3],
            yPos[3],
            driveInverted[3],
            steerInverted[3],
            encoderInverted[3]);
  }

  public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    }

    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          modules);
    }

    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          odometryStandardDeviation,
          visionStandardDeviation,
          modules);
    }
  }
}
