// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {

  private final SingleJointedArmSim sim;
  private final PIDController pid = new PIDController(5.0, 0, 0.1);

  private double targetRads = 0.0;
  private boolean closedLoop = false;

  public HoodIOSim() {
    sim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            45.0,
            0.3,
            0.2,
            Units.degreesToRadians(0),
            Units.degreesToRadians(60),
            true,
            Units.degreesToRadians(0));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double volts = 0.0;
    if (closedLoop) {
      volts = pid.calculate(sim.getAngleRads(), targetRads);
    }
    sim.setInputVoltage(volts);
    sim.update(0.02);

    inputs.connected = true;
    inputs.angleDegrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.appliedVolts = volts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setAngle(double angleDegrees) {
    closedLoop = true;
    targetRads = Units.degreesToRadians(angleDegrees);
  }

  @Override
  public void stop() {
    closedLoop = false;
  }

  @Override
  public void setPID(double kP, double kD, double kG, double kV, double kS) {
    pid.setPID(kP, 0, kD);
  }
}
