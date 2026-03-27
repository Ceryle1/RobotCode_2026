package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {

  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * @param name The configured name of the camera on the coprocessor.
   * @param robotToCamera 3-D transform from the robot's origin to this camera.
   */
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

      // ── Latest target tx/ty for auto-aim ───────────────────────────────
      if (result.hasTargets()) {
        PhotonTrackedTarget best = result.getBestTarget();
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(best.getYaw()), Rotation2d.fromDegrees(best.getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      if (!result.hasTargets()) continue;

      // ── Multi-tag PnP (preferred for autonomous) ────────────────────────
      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();

        // field → camera transform comes directly from PhotonVision's multi-tag solver
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;

        // field → robot:  apply the inverse of robotToCamera (i.e. cameraToRobot)
        Pose3d robotPose =
            new Pose3d().transformBy(fieldToCamera).transformBy(robotToCamera.inverse());

        // Average distance across ALL targets visible in this frame
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        double avgTagDistance = totalTagDistance / result.targets.size();

        tagIds.addAll(multitagResult.fiducialIDsUsed);

        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                robotPose,
                multitagResult.estimatedPose.ambiguity,
                multitagResult.fiducialIDsUsed.size(),
                avgTagDistance,
                PoseObservationType.PHOTONVISION_MULTITAG));

      } else {
        // ── Single-tag SolvePnP (auto-aim fallback) ──────────────────────
        // Use the best target to anchor a pose estimate off the known tag pose.
        PhotonTrackedTarget target = result.getBestTarget();

        // Skip if tag isn't in the layout (unknown field element)
        Optional<Pose3d> tagPoseOpt = aprilTagLayout.getTagPose(target.getFiducialId());
        if (tagPoseOpt.isEmpty()) continue;

        Pose3d tagPose = tagPoseOpt.get();
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        double tagDistance = cameraToTarget.getTranslation().getNorm();

        // field → tag (from layout) → camera (inverse of cameraToTarget) → robot (inverse of
        // robotToCamera)
        Pose3d robotPose =
            tagPose
                .transformBy(cameraToTarget.inverse()) // field → camera
                .transformBy(robotToCamera.inverse()); // field → robot

        tagIds.add((short) target.getFiducialId());

        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                robotPose,
                target.getPoseAmbiguity(),
                1,
                tagDistance,
                PoseObservationType.PHOTONVISION_SINGLETAG));
      }
    }

    // Flush to inputs
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
