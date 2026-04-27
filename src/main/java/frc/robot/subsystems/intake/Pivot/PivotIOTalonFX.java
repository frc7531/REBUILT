// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.Pivot;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

public class PivotIOTalonFX implements PivotIO {

  private final TalonFX talon;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final CANcoder tb;
  private final CANcoderConfiguration tbConfig = new CANcoderConfiguration();

  private final VoltageOut voltageRequest = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0).withUpdateFreqHz(0).withEnableFOC(true);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final MotionMagicVoltage motionRequest =
      new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0).withEnableFOC(true);

  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Angle> angle, tbAngle;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;

  private final Debouncer connectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public PivotIOTalonFX() {
    talon = new TalonFX(pivotMotorId, canbus);
    tb = new CANcoder(tbId, canbus);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0).withKG(0).withKS(0).withKV(0);
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    config.MotionMagic.MotionMagicAcceleration = 0.0;
    config.MotionMagic.MotionMagicJerk = 0.0;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(0.0);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(0.0);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.TorqueCurrent.PeakForwardTorqueCurrent = pivotStatorCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -pivotStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = pivotStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = pivotSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    config.Feedback.FeedbackRemoteSensorID = tbId;
    tbConfig.MagnetSensor.SensorDirection =
        tbInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> tb.setPosition(0));

    temp = talon.getDeviceTemp();
    angle = tb.getPosition();
    tbAngle = tb.getPosition();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, temp, angle, tbAngle, appliedVolts, supplyCurrent, torqueCurrent);
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon));

    PhoenixUtil.registerSignals(
        false, temp, angle, tbAngle, appliedVolts, supplyCurrent, torqueCurrent);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            temp, angle, tbAngle, appliedVolts, supplyCurrent, torqueCurrent);

    inputs.connected = connectedDebouncer.calculate(status.isOK());
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.angleDegrees = Units.rotationsToDegrees(angle.getValueAsDouble());
    inputs.tbAngleDegrees = Units.rotationsToDegrees(tbAngle.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  // @Override
  // public void setAngle(double angleDegrees) {
  //   double currentRot = angle.getValueAsDouble();
  //   double target = Units.degreesToRotations(angleDegrees);
  //   int slot = target > currentRot ? 0 : 1;

  //   talon.setControl(positionVoltageRequest.withPosition(target).withSlot(slot));
  // }

  @Override
  public void setAngle(double angleDegrees) {
    talon.setControl(motionRequest.withPosition(angleDegrees));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              config.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
            })
        .start();
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

  @Override
  public void stop() {
    talon.set(0);
  }
}
