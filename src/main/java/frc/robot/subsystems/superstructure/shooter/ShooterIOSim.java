// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

  private final FlywheelSim leftSim;
  private final FlywheelSim rightSim;
  private final PIDController pid = new PIDController(0.5, 0, 0);

  private double leftVolts = 0.0;
  private double rightVolts = 0.0;
  private boolean closedLoop = false;
  private double targetRPM = 0.0;

  public ShooterIOSim() {
    var motor = DCMotor.getKrakenX60Foc(1);
    leftSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, 0.004, 1.0), motor);
    rightSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, 0.004, 1.0), motor);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoop) {
      double leftRPM = leftSim.getAngularVelocityRPM();
      double rightRPM = rightSim.getAngularVelocityRPM();
      leftVolts = pid.calculate(leftRPM, targetRPM);
      rightVolts = pid.calculate(rightRPM, targetRPM);
    }

    leftSim.setInputVoltage(leftVolts);
    rightSim.setInputVoltage(rightVolts);
    leftSim.update(0.02);
    rightSim.update(0.02);

    inputs.leftConnected = true;
    inputs.leftVelocityRPM = leftSim.getAngularVelocityRPM();
    inputs.leftAppliedVolts = leftVolts;
    inputs.leftSupplyCurrentAmps = leftSim.getCurrentDrawAmps();
    inputs.leftTorqueCurrentAmps = leftSim.getCurrentDrawAmps();

    inputs.rightConnected = true;
    inputs.rightVelocityRPM = rightSim.getAngularVelocityRPM();
    inputs.rightAppliedVolts = rightVolts;
    inputs.rightSupplyCurrentAmps = rightSim.getCurrentDrawAmps();
    inputs.rightTorqueCurrentAmps = rightSim.getCurrentDrawAmps();
  }

  @Override
  public void runVelocity(double velocityRPM, double feedforward) {
    closedLoop = true;
    targetRPM = velocityRPM;
  }

  @Override
  public void runOpenLoop(double output) {
    closedLoop = false;
    leftVolts = output * 12.0;
    rightVolts = output * 12.0;
  }

  @Override
  public void stop() {
    closedLoop = false;
    leftVolts = 0;
    rightVolts = 0;
  }

  @Override
  public void setPID(double kP, double kD, double kV, double kS, double kA) {
    pid.setPID(kP, 0, kD);
  }
}
