// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.transfer;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
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

public class TransferIOTalonFX implements TransferIO {

  private final TalonFX talon;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageRequest = new VoltageOut(0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);

  private final StatusSignal<Temperature> temp;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;

  private final Debouncer connectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public TransferIOTalonFX() {
    talon = new TalonFX(transferMotorId, canbus);

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    config.Feedback.SensorToMechanismRatio = transferReduction;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.TorqueCurrent.PeakForwardTorqueCurrent = transferStatorCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -transferStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = transferStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = transferSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    temp = talon.getDeviceTemp();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, temp, velocity, appliedVolts, supplyCurrent, torqueCurrent);
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon));

    PhoenixUtil.registerSignals(false, temp, velocity, appliedVolts, supplyCurrent, torqueCurrent);
  }

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(temp, velocity, appliedVolts, supplyCurrent, torqueCurrent);

    inputs.connected = connectedDebouncer.calculate(status.isOK());
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.velocityRPM = velocity.getValueAsDouble() * 60;
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void runVelocity(double velocityRPM) {
    talon.setControl(velocityRequest.withVelocity(velocityRPM / 60));
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }

  @Override
  public void setPID(double kP, double kD, double kV, double kS) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.Slot0.kV = kV;
    config.Slot0.kS = kS;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }
}
