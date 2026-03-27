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

  /** Raw tx/ty angles to the best visible target. Used for simple auto-aim servoing. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /**
   * A full robot-pose estimate derived from one or more AprilTags.
   *
   * @param timestamp FPGA time of the camera frame (seconds)
   * @param pose Estimated robot pose in field space
   * @param ambiguity PnP ambiguity ratio (0 = perfect, 1 = degenerate)
   * @param tagCount Number of AprilTags used in this estimate
   * @param averageTagDistance Mean camera-to-tag distance across all used tags (meters)
   * @param type Which solver produced this estimate
   */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    /** Multi-tag PnP solve — highest accuracy, preferred for autonomous. */
    PHOTONVISION_MULTITAG,

    /** Single-tag SolvePnP — noisier; filtered more aggressively, but useful for auto-aim. */
    PHOTONVISION_SINGLETAG
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
