// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.zonecontroller.RebuiltZones;
import frc.robot.zonecontroller.ZoneController;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose Estimation
  private SwerveDrivePoseEstimator poseEstimator;
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  // Velocities
  private final AtomicReference<ChassisSpeeds> measuredChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> desiredChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());

  // Mechanism State
  private final AtomicReference<Double> shooterVelocityRPM = new AtomicReference<>(0.0);
  private final AtomicReference<Double> transferVelocityRPM = new AtomicReference<>(0.0);
  private final AtomicReference<Double> floorVelocityRPM = new AtomicReference<>(0.0);
  private final AtomicReference<Double> wallVelocityRPM = new AtomicReference<>(0.0);
  private final AtomicReference<Double> rollersVelocityRPM = new AtomicReference<>(0.0);
  private final AtomicReference<Double> hoodAngleDeg = new AtomicReference<>(0.0);
  private final AtomicReference<Double> pivotDeg = new AtomicReference<>(0.0);

  // Current draw tracking
  private final AtomicReference<Double> driveCurrentDraw = new AtomicReference<>(0.0);
  private final AtomicReference<Double> shooterCurrentDraw = new AtomicReference<>(0.0);
  private final AtomicReference<Double> transferCurrentDraw = new AtomicReference<>(0.0);
  private final AtomicReference<Double> floorCurrentDraw = new AtomicReference<>(0.0);
  private final AtomicReference<Double> wallCurrentDraw = new AtomicReference<>(0.0);
  private final AtomicReference<Double> hoodCurrentDraw = new AtomicReference<>(0.0);
  private final AtomicReference<Double> pivotCurrentDraw = new AtomicReference<>(0.0);
  private final AtomicReference<Double> rollersCurrentDraw = new AtomicReference<>(0.0);

  // Activity state
  private final AtomicReference<SuperstructureActivity> superstructureActivity =
      new AtomicReference<>(SuperstructureActivity.IDLE);
  private final AtomicReference<IntakeActivity> intakeActivity =
      new AtomicReference<>(IntakeActivity.IDLE);

  public enum SuperstructureActivity {
    IDLE,
    PREPARING,
    SHOOTING,
    INDEXING,
    UNCLOGGING,
    UNDEXING,
    FERRYING,
    STOWING
  }

  public enum IntakeActivity {
    IDLE,
    DEPLOYING,
    INTAKING,
    STOWING,
    FEEDING
  }

  // Shot parameters
  public enum ShotMode {
    AUTO,
    FORCE_HUB,
    FORCE_FERRY
  }

  private final AtomicReference<ShotMode> shotMode = new AtomicReference<>(ShotMode.AUTO);

  private RobotState() {}

  public void initPoseEstimator(SwerveDriveKinematics kinematics) {
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);
  }

  public void addOdometryObservation(
      double timestamp,
      Rotation2d gyroAngle,
      boolean gyroConnected,
      SwerveModulePosition[] modulePositions,
      SwerveDriveKinematics kinematics) {
    if (gyroConnected) {
      rawGyroRotation = gyroAngle;
    } else {
      var twist = kinematics.toTwist2d(lastModulePositions, modulePositions);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }
    lastModulePositions = modulePositions;
    poseEstimator.updateWithTime(timestamp, rawGyroRotation, modulePositions);
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator != null ? poseEstimator.getEstimatedPosition() : Pose2d.kZero;
  }

  public Rotation2d getRotation() {
    return getEstimatedPose().getRotation();
  }

  // Velocity Methods

  public void setMeasuredSpeeds(ChassisSpeeds speeds) {
    measuredChassisSpeeds.set(speeds);
  }

  public void setDesiredSpeeds(ChassisSpeeds speeds) {
    desiredChassisSpeeds.set(speeds);
  }

  public ChassisSpeeds getMeasuredSpeeds() {
    return measuredChassisSpeeds.get();
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(measuredChassisSpeeds.get(), getRotation());
  }

  // Mechanism state setters

  public void setShooterVelocityRPM(double rpm) {
    shooterVelocityRPM.set(rpm);
  }

  public void setTransferVelocityRPM(double rpm) {
    transferVelocityRPM.set(rpm);
  }

  public void setFloorVelocityRPM(double rpm) {
    floorVelocityRPM.set(rpm);
  }

  public void setWallVelocityRPM(double rpm) {
    wallVelocityRPM.set(rpm);
  }

  public void setRollersVelocityRPM(double rpm) {
    rollersVelocityRPM.set(rpm);
  }

  public void setHoodAngleDeg(double deg) {
    hoodAngleDeg.set(deg);
  }

  public void setPivotDeg(double deg) {
    pivotDeg.set(deg);
  }

  public double getShooterVelocityRPM() {
    return shooterVelocityRPM.get();
  }

  public double getTransferVelocityRPM() {
    return transferVelocityRPM.get();
  }

  public double getFloorVelocityRPM() {
    return floorVelocityRPM.get();
  }

  public double getWallVelocityRPM() {
    return wallVelocityRPM.get();
  }

  public double getRollersVelocityRPM() {
    return rollersVelocityRPM.get();
  }

  public double getHoodAngleDeg() {
    return hoodAngleDeg.get();
  }

  public double getPivotDeg() {
    return pivotDeg.get();
  }

  // Current draw setters

  public void setDriveCurrentDraw(double amps) {
    driveCurrentDraw.set(amps);
  }

  public void setShooterCurrentDraw(double amps) {
    shooterCurrentDraw.set(amps);
  }

  public void setHoodCurrentDraw(double amps) {
    hoodCurrentDraw.set(amps);
  }

  public void setTransferCurrentDraw(double amps) {
    transferCurrentDraw.set(amps);
  }

  public void setWallCurrentDraw(double amps) {
    wallCurrentDraw.set(amps);
  }

  public void setFloorCurrentDraw(double amps) {
    floorCurrentDraw.set(amps);
  }

  public void setRollersCurrentDraw(double amps) {
    rollersCurrentDraw.set(amps);
  }

  public void setPivotCurrentDraw(double amps) {
    pivotCurrentDraw.set(amps);
  }

  public double getTotalCurrentDraw() {
    return driveCurrentDraw.get()
        + shooterCurrentDraw.get()
        + hoodCurrentDraw.get()
        + transferCurrentDraw.get()
        + wallCurrentDraw.get()
        + floorCurrentDraw.get()
        + rollersCurrentDraw.get()
        + pivotCurrentDraw.get();
  }

  // Activity State

  public void setSuperstructureActivity(SuperstructureActivity activity) {
    superstructureActivity.set(activity);
  }

  public SuperstructureActivity getSuperstructureActivity() {
    return superstructureActivity.get();
  }

  public void setIntakeActivity(IntakeActivity activity) {
    intakeActivity.set(activity);
  }

  public IntakeActivity getIntakeActivity() {
    return intakeActivity.get();
  }

  // Shot mode

  public void setShotMode(ShotMode mode) {
    shotMode.set(mode);
  }

  public ShotMode getShotMode() {
    return shotMode.get();
  }

  public Translation2d getAimTarget() {
    return ZoneController.getInstance().getAimTarget();
  }

  public boolean shouldFerry() {
    return ZoneController.getInstance().shouldFerry();
  }

  public boolean shouldStowHood() {
    return ZoneController.getInstance().shouldStowHood();
  }

  public boolean canScoreFromHere() {
    Translation2d pos = getEstimatedPose().getTranslation();
    boolean inAllianceZone =
        frc.robot.zonecontroller.RebuiltZones.allianceZone().containsPoint(pos);
    SuperstructureActivity activity = getSuperstructureActivity();
    boolean mechanismReady =
        activity == SuperstructureActivity.PREPARING || activity == SuperstructureActivity.SHOOTING;
    return inAllianceZone && mechanismReady;
  }

  /** Returns true if the robot is in the alliance zone (valid scoring area). */
  public boolean isInAllianceZone() {
    Translation2d pos = getEstimatedPose().getTranslation();
    return RebuiltZones.allianceZone.containsPoint(pos);
  }

  /** Returns true if the robot is in the neutral zone. */
  public boolean isInNeutralZone() {
    Translation2d pos = getEstimatedPose().getTranslation();
    return RebuiltZones.neutralZone().containsPoint(pos);
  }

  private final Field2d field = new Field2d();

  public void updateLogger() {
    Logger.recordOutput("RobotState/ShotMode", shotMode.get().toString());
    Logger.recordOutput("RobotState/ShooterVelocityRPM", getShooterVelocityRPM());
    Logger.recordOutput("RobotState/TransferVelocityRPM", getTransferVelocityRPM());
    Logger.recordOutput("RobotState/FloorVelocityRPM", getFloorVelocityRPM());
    Logger.recordOutput("RobotState/WallVelocityRPM", getWallVelocityRPM());
    Logger.recordOutput("RobotState/HoodAngleDeg", getHoodAngleDeg());
    Logger.recordOutput("RobotState/PivotDeg", getPivotDeg());

    Logger.recordOutput("RobotState/Current/Drive", driveCurrentDraw.get());
    Logger.recordOutput("RobotState/Current/Shooter", shooterCurrentDraw.get());
    Logger.recordOutput("RobotState/Current/Hood", hoodCurrentDraw.get());
    Logger.recordOutput("RobotState/Current/Wall", wallCurrentDraw.get());
    Logger.recordOutput("RobotState/Current/Transfer", transferCurrentDraw.get());
    Logger.recordOutput("RobotState/Current/Floor", floorCurrentDraw.get());
    Logger.recordOutput("RobotState/Current/Pivot", pivotCurrentDraw.get());
    Logger.recordOutput("RobotState/Current/Rollers", rollersCurrentDraw.get());
    Logger.recordOutput("RobotState/Current/Total", getTotalCurrentDraw());

    Logger.recordOutput(
        "RobotState/Activity/Superstructure", superstructureActivity.get().toString());
    Logger.recordOutput("RobotState/Activity/Intake", intakeActivity.get().toString());

    SmartDashboard.putData("Field", field);

    field.setRobotPose(RobotState.getInstance().getEstimatedPose());
  }

  public record ShotParameters(double rpm, double hoodDeg, boolean isFerry) {}
}
