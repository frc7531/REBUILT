// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConstants {

  //   private static final LoggedTunableNumber leftCamRoll =
  //       new LoggedTunableNumber("Vision/leftRoll", 5.0);
  //   private static final LoggedTunableNumber leftCamPitch =
  //       new LoggedTunableNumber("Vision/leftPitch", 25.0);
  //   private static final LoggedTunableNumber leftCamYaw =
  //       new LoggedTunableNumber("Vision/leftYaw", 8.0);

  //   private static final LoggedTunableNumber rightCamRoll =
  //       new LoggedTunableNumber("Vision/rightRoll", 6.6);
  //   private static final LoggedTunableNumber rightCamPitch =
  //       new LoggedTunableNumber("Vision/rightPitch", 27.4);
  //   private static final LoggedTunableNumber rightCamYaw =
  //       new LoggedTunableNumber("Vision/rightYaw", 12.0);

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String leftCamName = "leftCam";
  public static String rightCamName = "rightCam";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  //   public static Transform3d leftCamTranslation =
  //       new Transform3d(
  //           Units.inchesToMeters(-11.25),
  //           Units.inchesToMeters(-1.125),
  //           Units.inchesToMeters(8.625),
  //           new Rotation3d(
  //               Units.degreesToRadians(leftCamRoll.get()),
  //               Units.degreesToRadians(leftCamPitch.get()),
  //               Units.degreesToRadians(leftCamYaw.get()) + Units.degreesToRadians(180)));
  //   public static Transform3d RightCamTranslation =
  //       new Transform3d(
  //           Units.inchesToMeters(-11.25),
  //           Units.inchesToMeters(-1.125),
  //           Units.inchesToMeters(8.375),
  //           new Rotation3d(
  //               Units.degreesToRadians(rightCamRoll.get()),
  //               Units.degreesToRadians(rightCamPitch.get()),
  //               Units.degreesToRadians(-rightCamYaw.get()) + Units.degreesToRadians(180)));
  //   public static Transform3d leftCamTranslation =
  //       new Transform3d(
  //           Units.inchesToMeters(-1.125),
  //           Units.inchesToMeters(-11.25),
  //           Units.inchesToMeters(8.625),
  //           new Rotation3d(
  //               Units.degreesToRadians(leftCamRoll.get()),
  //               Units.degreesToRadians(leftCamPitch.get()),
  //               Units.degreesToRadians(leftCamYaw.get()) + Units.degreesToRadians(180)));
  //   public static Transform3d RightCamTranslation =
  //       new Transform3d(
  //           Units.inchesToMeters(-1.125),
  //           Units.inchesToMeters(11.25),
  //           Units.inchesToMeters(8.375),
  //           new Rotation3d(
  //               Units.degreesToRadians(rightCamRoll.get()),
  //               Units.degreesToRadians(rightCamPitch.get()),
  //               Units.degreesToRadians(-rightCamYaw.get()) + Units.degreesToRadians(180)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter dxistance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
