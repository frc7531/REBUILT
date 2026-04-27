// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoChooser {
  private final LoggedDashboardChooser<Command> chooser;

  public AutoChooser(Drive drive, Intake intake, Superstructure superstructure) {
    registerNamedCommands(drive, intake, superstructure);

    chooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    chooser.addDefaultOption("Do Nothing", Commands.none());

    if (Constants.tuningMode) {
      chooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
      chooser.addOption(
          "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
      chooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      chooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      chooser.addOption(
          "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      chooser.addOption(
          "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  private void registerNamedCommands(Drive drive, Intake intake, Superstructure superstructure) {
    NamedCommands.registerCommand(
        "Intake Deploy", Commands.runOnce(() -> intake.setGoal(Intake.Goal.DEPLOY)));
    NamedCommands.registerCommand(
        "Intake", Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)));
    NamedCommands.registerCommand(
        "Intake Stow", Commands.runOnce(() -> intake.setGoal(Intake.Goal.STOW)));
    NamedCommands.registerCommand(
        "Intake Feed", Commands.runOnce(() -> intake.setGoal(Intake.Goal.FEED)));
    NamedCommands.registerCommand(
        "Intake Idle", Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)));
    NamedCommands.registerCommand(
        "Shoot", Commands.runOnce(() -> superstructure.setGoal(Superstructure.Goal.SHOOT)));
    NamedCommands.registerCommand(
        "Stop Shooting", Commands.runOnce(() -> superstructure.setGoal(Superstructure.Goal.IDLE)));
    NamedCommands.registerCommand("Auto Aim", DriveCommands.aimAtTarget(drive));
  }

  public Command getSelected() {
    return chooser.get();
  }
}
