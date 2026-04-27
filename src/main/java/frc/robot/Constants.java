// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static RobotType robot = RobotType.COMPBOT;
  public static final boolean tuningMode = false;

  public static final double loopPeriodSecs = 0.02;
  public static final double loopPeriodWatchdogSecs = 0.2;

  public static final Mode getMode() {
    return switch (robot) {
      case COMPBOT, PRACBOT -> RobotBase.isReal()
          ? Mode.REAL
          : (RobotBase.isSimulation() ? Mode.SIM : Mode.SIM);
      case SIMBOT -> Mode.SIM;
    };
  }

  @SuppressWarnings("resource")
  public static final RobotType getRobot() {
    if (RobotBase.isReal() && robot == RobotType.SIMBOT) {
      new Alert(
              "Invalid robot configuration; RobotType set as SIMBOT when robot is real. Using COMPBOT as default",
              AlertType.kError)
          .set(true);
      robot = RobotType.COMPBOT;
    }
    return robot;
  }

  public enum Mode {
    REAL,
    SIM,
    REPLAY,
  }

  public enum RobotType {
    COMPBOT,
    PRACBOT,
    SIMBOT,
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robot == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robot);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robot != RobotType.COMPBOT || tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }
}
