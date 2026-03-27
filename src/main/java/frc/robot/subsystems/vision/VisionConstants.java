package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names — must match names configured on coprocessor
  public static String CAMERA_FRONT_NAME = "camera_0";
  public static String CAMERA_BACK_NAME = "camera_1";

  // Robot-to-camera transforms (meters + radians)
  // Roll, Pitch, Yaw in Rotation3d
  public static Transform3d robotToCamera0 =
      new Transform3d(0.29, 0.247, 0.47, new Rotation3d(0.0, 1.22, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Pose filtering thresholds
  // Poses with ambiguity above this are rejected when only 1 tag is visible
  public static double maxAmbiguity = 0.4;
  // Poses where the robot Z is further than this from 0 are rejected (robot is on the floor)
  public static double maxZError = 0.3; // meters — was 10, which is too lenient

  // Standard deviation baselines at 1 meter distance with 1 tag.
  // These scale automatically: stdDev = baseline * (distance² / tagCount)
  public static double linearStdDevBaseline = 0.02; // meters
  public static double angularStdDevBaseline = 0.06; // radians

  // Per-camera trust multipliers — increase to trust a camera less
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0 (front PhotonVision)
        1.0, // Camera 1 (back PhotonVision)
        1.0 // Camera 2 (QuestNav) — decrease to trust Quest more, increase to trust it less
      };

  // QuestNav base std dev factor — overrides the distance-based formula entirely.
  // Lower = trust QuestNav more relative to wheel odometry.
  public static double questNavStdDevFactor = 0.1;
}
