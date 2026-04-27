// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlsXbox implements Controls {

  private final CommandXboxController controller;

  public ControlsXbox(int port) {
    controller = new CommandXboxController(port);
  }

  @Override
  public double getForward() {
    return -controller.getLeftY();
  }

  @Override
  public double getStrafe() {
    return -controller.getLeftX();
  }

  @Override
  public double getTurn() {
    return -controller.getRightX();
  }

  @Override
  public Trigger resetGyro() {
    return controller.start().and(controller.back());
  }

  @Override
  public Trigger shoot() {
    return controller.rightTrigger();
  }

  @Override
  public Trigger intake() {
    return controller.leftTrigger();
  }

  @Override
  public Trigger stowIntake() {
    return controller.leftBumper();
  }

  @Override
  public Trigger stowHood() {
    return controller.leftBumper().and(controller.rightBumper());
  }

  @Override
  public Trigger unclog() {
    return controller.x();
  }

  @Override
  public Trigger forceHub() {
    return controller.y();
  }

  @Override
  public Trigger forceFerryRight() {
    return controller.b();
  }

  @Override
  public Trigger forceFerryLeft() {
    return controller.a();
  }

  @Override
  public Trigger spit() {
    return controller.povDown();
  }

  @Override
  public Trigger shootBackup() {
    return controller.povRight();
  }

  @Override
  public Trigger deployTest() {
    return controller.povLeft();
  }

  @Override
  public Trigger alignTest() {
    return controller.povUp();
  }
}
