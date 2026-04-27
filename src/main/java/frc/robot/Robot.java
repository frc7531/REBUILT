// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.util.LoggedTracer;
import frc.robot.util.VirtualSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private static final double lowBatteryVoltage = 11.0;
  private static final double lowBatteryDisabledTime = 2.0;

  private Command autonomousCommand;
  private double autoStart;
  private boolean autoMessagePrinted;
  private RobotContainer robotContainer;

  private final Timer disabledTimer = new Timer();
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, turn off the robot or replace the battery to avoid damage",
          AlertType.kWarning);

  public Robot() {
    super(Constants.loopPeriodSecs);

    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommited changes";
          default -> "Unkown";
        });

    switch (Constants.getMode()) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        String inPath = LogFileUtil.findReplayLog();
        String outPath = LogFileUtil.addPathSuffix(inPath, "_sim");
        Logger.setReplaySource(new WPILOGReader(inPath));
        Logger.addDataReceiver(new WPILOGWriter(outPath));
        break;
    }

    setUseTiming(Constants.getMode() != Mode.REPLAY);

    SignalLogger.enableAutoLogging(false);

    Logger.start();

    DriverStation.silenceJoystickConnectionWarning(true);

    var mathShared = MathSharedStore.getMathShared();
    MathSharedStore.setMathShared(
        new MathShared() {
          @Override
          public void reportError(String error, StackTraceElement[] stackTrace) {
            if (error.startsWith("x and y components of Rotation2d are zero")) return;
            mathShared.reportError(error, stackTrace);
          }

          @Override
          public void reportUsage(MathUsageId id, int count) {
            mathShared.reportUsage(id, count);
          }

          @Override
          public double getTimestamp() {
            return mathShared.getTimestamp();
          }
        });

    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    RoboRioSim.setTeamNumber(10219);
    if (Constants.robot == RobotType.SIMBOT) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }
    disabledTimer.reset();

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();
    VirtualSubsystem.periodicAll();
    CommandScheduler.getInstance().run();
    LoggedTracer.record("Commands");

    robotContainer.periodic();

    if (autonomousCommand != null) {
      if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }

    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() > 0.0
        && RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      lowBatteryAlert.set(true);
    }

    Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());

    LoggedTracer.record("RobotPeriodic");
  }

  public static boolean showHardwareAlerts() {
    return Constants.getMode() != Mode.SIM && Timer.getTimestamp() > 30.0;
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autoStart = Timer.getTimestamp();
    autoMessagePrinted = false;
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
