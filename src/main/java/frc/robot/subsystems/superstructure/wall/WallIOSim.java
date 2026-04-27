// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.wall;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.superstructure.SuperstructureConstants;

public class WallIOSim implements WallIO {

  private final FlywheelSim sim;
  private final PIDController pid = new PIDController(0.5, 0, 0);

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double targetRPM = 0.0;

  public WallIOSim() {
    var motor = DCMotor.getKrakenX60Foc(1);
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                motor, 0.005, SuperstructureConstants.rollerWallReduction),
            motor);
  }

  @Override
  public void updateInputs(WallIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = pid.calculate(sim.getAngularVelocityRPM(), targetRPM);
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.connected = true;
    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void runVelocity(double velocityRPM) {
    closedLoop = true;
    targetRPM = velocityRPM;
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }

  @Override
  public void setPID(double kP, double kD, double kV, double kS) {
    pid.setPID(kP, 0, kD);
  }
}
