// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.hood;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX implements HoodIO {

  private final TalonFX talon;
  private final CANcoder tb;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final CANcoderConfiguration tbConfig = new CANcoderConfiguration();

  private final PositionTorqueCurrentFOC positionRequest =
      new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0).withUpdateFreqHz(0).withEnableFOC(true);
  private final MotionMagicVoltage motionRequest =
      new MotionMagicVoltage(0).withUpdateFreqHz(0).withEnableFOC(true);

  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Angle> angle;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final Debouncer connectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public HoodIOTalonFX() {
    talon = new TalonFX(hoodMotorId, canbus);
    tb = new CANcoder(tbId, canbus);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0).withKD(0).withKG(0);
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    config.MotionMagic.MotionMagicAcceleration = 0.0;
    config.MotionMagic.MotionMagicJerk = 0.0;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.TorqueCurrent.PeakForwardTorqueCurrent = hoodStatorCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -hoodStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = hoodStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = hoodSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    tbConfig.MagnetSensor.SensorDirection =
        tbInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    config.Feedback.FeedbackRemoteSensorID = tbId;
    config.Feedback.SensorToMechanismRatio = hoodReduction;
    config.Feedback.RotorToSensorRatio = hoodReduction;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> tb.getConfigurator().apply(tbConfig, 0.25));
    tryUntilOk(5, () -> talon.setPosition(0.0));
    tryUntilOk(5, () -> tb.setPosition(0.0));

    temp = talon.getDeviceTemp();
    angle = talon.getPosition();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, temp, angle, appliedVolts, supplyCurrent, torqueCurrent);
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon));

    PhoenixUtil.registerSignals(false, temp, angle, appliedVolts, supplyCurrent, torqueCurrent);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(temp, angle, appliedVolts, supplyCurrent, torqueCurrent);

    inputs.connected = connectedDebouncer.calculate(status.isOK());
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.angleDegrees = Units.rotationsToDegrees(angle.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
  }

  @Override
  public void setAngle(double angleDegrees) {
    talon.setControl(motionRequest.withPosition(Units.degreesToRotations(angleDegrees)));
  }

  @Override
  public void stop() {
    talon.set(0);
  }

  @Override
  public void setPID(double kP, double kD, double kG, double kV, double kS) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.Slot0.kG = kG;
    config.Slot0.kV = kV;
    config.Slot0.kS = kS;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setMotionMagic(double cruiseVelo, double acceleration, double jerk) {
    config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelo;
    config.MotionMagic.MotionMagicAcceleration = acceleration;
    config.MotionMagic.MotionMagicJerk = jerk;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setMinMax(double minDeg, double maxDeg) {
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxDeg;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minDeg;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }
}
