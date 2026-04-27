// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controls {
  double getForward();

  double getStrafe();

  double getTurn();

  Trigger resetGyro();

  Trigger shoot();

  Trigger intake();

  Trigger stowIntake();

  Trigger stowHood();

  Trigger unclog();

  Trigger forceHub();

  Trigger forceFerryLeft();

  Trigger forceFerryRight();

  Trigger spit();

  Trigger shootBackup();

  Trigger deployTest();

  Trigger alignTest();
}
