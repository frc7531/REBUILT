// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.Pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.IntakeConstants;

public class PivotIOSim implements PivotIO {

  private final SingleJointedArmSim sim;
  private final PIDController pid = new PIDController(5.0, 0, 0.1);

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double targetRads = 0.0;

  public PivotIOSim() {
    sim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            IntakeConstants.pivotReduction,
            0.5,
            0.3,
            Units.degreesToRadians(-180),
            Units.degreesToRadians(10),
            true,
            0.0);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = pid.calculate(sim.getAngleRads(), targetRads);
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.connected = true;
    inputs.angleDegrees = Units.radiansToDegrees(sim.getAngleRads());
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
  public void setAngle(double angleDegrees) {
    closedLoop = true;
    targetRads = Units.degreesToRadians(angleDegrees);
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }

  @Override
  public void setPID(double kP, double kD, double kG, double kV, double kS) {}
}
