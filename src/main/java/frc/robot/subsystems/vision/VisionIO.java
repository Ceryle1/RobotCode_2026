package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Raw tx/ty angles to the best visible target. Useful for simple vision servoing. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** A full robot-pose estimate derived from one or more AprilTags. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    PHOTONVISION_MULTITAG, // PnP solve using multiple AprilTags — most accurate
    PHOTONVISION_SINGLETAG, // Single AprilTag solve — noisier, filtered more aggressively
    QUESTNAV // Meta Quest absolute pose — high-frequency, no AprilTag dependency
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
