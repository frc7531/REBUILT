// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX leftTalon;
  private final TalonFX rightTalon;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);

  private final StatusSignal<Temperature> leftTemp, rightTemp;
  private final StatusSignal<AngularVelocity> leftVelocity, rightVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts, rightAppliedVolts;
  private final StatusSignal<Current> leftSupply, rightSupply;
  private final StatusSignal<Current> leftTorque, rightTorque;

  private final Debouncer leftConnected = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rightConnected = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ShooterIOTalonFX() {
    leftTalon = new TalonFX(leftShooterId, canbus);
    rightTalon = new TalonFX(rightShooterId, canbus);

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    config.Feedback.SensorToMechanismRatio = shooterReduction;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.TorqueCurrent.PeakForwardTorqueCurrent = shooterStatorCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -shooterStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = shooterStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = shooterSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(config, 0.25));
    tryUntilOk(
        5,
        () ->
            rightTalon
                .getConfigurator()
                .apply(
                    config, 0.25)); // .MotorOutput.withInverted(InvertedValue.Clockwise_Positive)

    leftTemp = leftTalon.getDeviceTemp();
    leftVelocity = leftTalon.getVelocity();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftSupply = leftTalon.getSupplyCurrent();
    leftTorque = leftTalon.getTorqueCurrent();

    rightTemp = rightTalon.getDeviceTemp();
    rightVelocity = rightTalon.getVelocity();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightSupply = rightTalon.getSupplyCurrent();
    rightTorque = rightTalon.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftTemp,
        leftVelocity,
        leftAppliedVolts,
        leftSupply,
        leftTorque,
        rightTemp,
        rightVelocity,
        rightAppliedVolts,
        rightSupply,
        rightTorque);
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(leftTalon, rightTalon));

    PhoenixUtil.registerSignals(
        false,
        leftTemp,
        leftVelocity,
        leftAppliedVolts,
        leftSupply,
        leftTorque,
        rightTemp,
        rightVelocity,
        rightAppliedVolts,
        rightSupply,
        rightTorque);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var ls =
        BaseStatusSignal.refreshAll(
            leftTemp, leftVelocity, leftAppliedVolts, leftSupply, leftTorque);
    var rs =
        BaseStatusSignal.refreshAll(
            rightTemp, rightVelocity, rightAppliedVolts, rightSupply, rightTorque);

    inputs.leftConnected = leftConnected.calculate(ls.isOK());
    inputs.leftTempCelsius = leftTemp.getValueAsDouble();
    inputs.leftVelocityRPM = leftVelocity.getValueAsDouble() * 60;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupply.getValueAsDouble();
    inputs.leftTorqueCurrentAmps = leftTorque.getValueAsDouble();

    inputs.rightConnected = rightConnected.calculate(rs.isOK());
    inputs.rightTempCelsius = rightTemp.getValueAsDouble();
    inputs.rightVelocityRPM = rightVelocity.getValueAsDouble() * 60;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupply.getValueAsDouble();
    inputs.rightTorqueCurrentAmps = rightTorque.getValueAsDouble();
  }

  @Override
  public void runOpenLoop(double output) {
    leftTalon.set(output);
    rightTalon.set(-output);
  }

  @Override
  public void runVelocity(double velocityRPM, double feedforward) {
    leftTalon.setControl(
        velocityRequest.withVelocity(velocityRPM / 60).withFeedForward(feedforward));
    rightTalon.setControl(
        velocityRequest.withVelocity(-velocityRPM / 60).withFeedForward(feedforward));
  }

  @Override
  public void stop() {
    runOpenLoop(0);
  }

  @Override
  public void setPID(double kP, double kD, double kV, double kS, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.Slot0.kV = kV;
    config.Slot0.kS = kS;
    config.Slot0.kA = kA;
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> rightTalon.getConfigurator().apply(config, 0.25));
  }
}
