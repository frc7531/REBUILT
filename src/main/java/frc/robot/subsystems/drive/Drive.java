// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggedTunableNumber;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  public enum Goal {
    TELEOP,
    AIM_AND_DRIVE,
    SHOOT_LOCKED,
    AUTO_PATH,
    CHARACTERIZE,
    X_LOCK
  }

  public enum State {
    DRIVING,
    DRIVING_REDUCED,
    AIMING,
    SHOOT_LOCKED,
    PATHING,
    PATHING_REDUCED,
    X_LOCKED,
    CHARACTERIZING,
    DISABLED
  }

  // Speed limit tunables
  private static final LoggedTunableNumber speedFull =
      new LoggedTunableNumber("Drive/SpeedLimit/Full", 1.0);
  private static final LoggedTunableNumber speedReduced =
      new LoggedTunableNumber("Drive/SpeedLimit/Reduced", 0.7);
  private static final LoggedTunableNumber speedAiming =
      new LoggedTunableNumber("Drive/SpeedLimit/Aiming", 0.4);
  private static final LoggedTunableNumber speedShootLocked =
      new LoggedTunableNumber("Drive/SpeedLimit/ShootLocked", 0.25);
  private static final LoggedTunableNumber speedPathReduced =
      new LoggedTunableNumber("Drive/SpeedLimit/PathReduced", 0.85);

  // Constants
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  private static final double ROBOT_MASS_KG = 60.6226203;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 2.0;

  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  // Fields
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

  @Getter private Goal goal = Goal.TELEOP;
  @Getter private State state = State.DRIVING;

  // Constructor

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);
    PhoenixOdometryThread.getInstance().start();
    RobotState.getInstance().initPoseEstimator(kinematics);

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) ->
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (s) -> Logger.recordOutput("Drive/SysIdState", s.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  // Goal setter

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  // Periodic

  @Override
  public void periodic() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Odometry -> RobotState
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    for (int i = 0; i < sampleTimestamps.length; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      for (int m = 0; m < 4; m++) {
        modulePositions[m] = modules[m].getOdometryPositions()[i];
      }
      RobotState.getInstance()
          .addOdometryObservation(
              sampleTimestamps[i],
              gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : Rotation2d.kZero,
              gyroInputs.connected,
              modulePositions,
              kinematics);
    }

    RobotState.getInstance().setMeasuredSpeeds(getChassisSpeeds());

    double totalCurrent = 0.0;
    for (var module : modules) {
      totalCurrent += module.getDriveCurrentAmps() + module.getTurnCurrentAmps();
    }
    RobotState.getInstance().setDriveCurrentDraw(totalCurrent);

    // Evaluate state
    state = evaluateState();

    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);

    Logger.recordOutput("Drive/Goal", goal.toString());
    Logger.recordOutput("Drive/State", state.toString());
    Logger.recordOutput("Drive/SpeedMultiplier", getSpeedMultiplier());
  }

  // State machine

  private State evaluateState() {
    if (DriverStation.isDisabled()) return State.DISABLED;

    if (goal == Goal.X_LOCK) return State.X_LOCKED;
    if (goal == Goal.CHARACTERIZE) return State.CHARACTERIZING;

    boolean robotBusy = isRobotDrawingHeavyCurrent();

    return switch (goal) {
      case TELEOP -> robotBusy ? State.DRIVING_REDUCED : State.DRIVING;
      case AIM_AND_DRIVE -> State.AIMING;
      case SHOOT_LOCKED -> State.SHOOT_LOCKED;
      case AUTO_PATH -> robotBusy ? State.PATHING_REDUCED : State.PATHING;
      default -> State.DRIVING;
    };
  }

  private boolean isRobotDrawingHeavyCurrent() {
    var ssActivity = RobotState.getInstance().getSuperstructureActivity();
    var intakeActivity = RobotState.getInstance().getIntakeActivity();

    return ssActivity == RobotState.SuperstructureActivity.SHOOTING
        || ssActivity == RobotState.SuperstructureActivity.PREPARING
        || ssActivity == RobotState.SuperstructureActivity.INDEXING
        || intakeActivity == RobotState.IntakeActivity.INTAKING;
  }

  @AutoLogOutput(key = "Drive/SpeedMultiplier")
  public double getSpeedMultiplier() {
    return switch (state) {
      case DRIVING -> speedFull.get();
      case DRIVING_REDUCED -> speedReduced.get();
      case AIMING -> speedAiming.get();
      case SHOOT_LOCKED -> speedShootLocked.get();
      case PATHING -> speedFull.get();
      case PATHING_REDUCED -> speedPathReduced.get();
      case X_LOCKED, DISABLED, CHARACTERIZING -> speedFull.get();
    };
  }

  // Velocity (applies speed multiplier to ALL callers)

  public void runVelocity(ChassisSpeeds speeds) {
    double multiplier = getSpeedMultiplier();
    ChassisSpeeds scaled =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond * multiplier,
            speeds.vyMetersPerSecond * multiplier,
            speeds.omegaRadiansPerSecond);

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(scaled, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // SysId

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  // Getters

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return RobotState.getInstance().getEstimatedPose();
  }

  public Rotation2d getRotation() {
    return RobotState.getInstance().getRotation();
  }

  public ChassisSpeeds getFieldVelocity() {
    return RobotState.getInstance().getFieldVelocity();
  }

  public void setPose(Pose2d pose) {
    RobotState.getInstance().resetPose(pose);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    RobotState.getInstance()
        .addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
