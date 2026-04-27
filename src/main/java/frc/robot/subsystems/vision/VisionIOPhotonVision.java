// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      }

      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();

        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Pose3d cameraPose = new Pose3d().plus(fieldToCamera);
        Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());

        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        tagIds.addAll(multitagResult.fiducialIDsUsed);

        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                robotPose,
                multitagResult.estimatedPose.ambiguity,
                multitagResult.fiducialIDsUsed.size(),
                totalTagDistance / result.targets.size(),
                PoseObservationType.PHOTONVISION));

      } else if (!result.targets.isEmpty()) {
        var target = result.targets.get(0);

        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Pose3d cameraPose = tagPose.get().transformBy(target.bestCameraToTarget.inverse());
          Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());

          tagIds.add((short) target.fiducialId);

          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(),
                  robotPose,
                  target.poseAmbiguity,
                  1,
                  target.bestCameraToTarget.getTranslation().getNorm(),
                  PoseObservationType.PHOTONVISION));
        }
      }
    }

    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
