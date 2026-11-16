// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Intake extends SubsystemBase {
  /** Creates a new SS_Intake. */
  public TalonFX pivot = new TalonFX(20, "CANivore");

  public TalonFX roller = new TalonFX(25, "CANivore");
  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTableEntry intakeEntry = inst.getTable("Intake").getEntry("Rollers");
  public NetworkTableEntry pivotEntry = inst.getTable("Intake").getEntry("Pivot Status");
  public boolean toggle = false;

  public SS_Intake() {
    pivot.setPosition(0);
    intakeEntry.setBoolean(false);
    pivotEntry.setString("Up");
  }

  public void intakeRollersOn(double speed) {
    roller.set(speed);
  }

  public void intakeRollersOnNoTimer(double speed) {
    roller.set(speed);
  }

  public void stopIntakeRollers() {
    roller.set(0.0);
  }

  public void intakeUnfold() {
    pivot.set(0.2);
  }

  public void PivotStop() {
    pivot.set(0);
  }

  public void intakeFold() {
    pivot.set(-0.2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeEntry.setBoolean(toggle);
    if (pivot.getPosition().getValueAsDouble() > 14) {
      pivotEntry.setString("Down");
    } else if (pivot.getPosition().getValueAsDouble() < 0.5) {
      pivotEntry.setString("Up");
    } else {
      pivotEntry.setString("Inbetween");
    }
  }
}
