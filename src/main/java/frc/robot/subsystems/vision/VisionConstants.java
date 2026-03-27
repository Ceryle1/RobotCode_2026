package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names — must match names configured on coprocessor
  public static String CAMERA_FRONT_NAME = "Arducam_Front";
  public static String CAMERA_BACK_NAME = "Arducam_Side";

  // Robot-to-camera transforms (meters + radians).
  // Rotation3d order: Roll, Pitch, Yaw
  public static Transform3d robotToCamera0 =
      new Transform3d(-0.25, -0.227, 0.47, new Rotation3d(0.0, 1.22, Math.PI));

  public static Transform3d robotToCamera1 =
      new Transform3d(-0.15, -0.243, 0.42, new Rotation3d(0.0, -0.4, Units.degreesToRadians(-90)));

  // ── Pose-rejection thresholds ────────────────────────────────────────────

  /** Single-tag estimates with ambiguity above this are discarded. */
  public static double maxAmbiguity = 0.1;

  /** Estimates where the robot Z deviates more than this (meters) are discarded. */
  public static double maxZError = 0.2;

  // ── Standard-deviation baselines ─────────────────────────────────────────
  // Final stdDev = baseline * (averageDistance² / tagCount) * cameraFactor
  // These apply to BOTH translation axes; angular gets its own baseline.

  /** Linear (x/y) baseline std dev at 1 m with 1 tag visible. */
  public static double linearStdDevBaseline = 0.02; // meters

  /** Angular (heading) baseline std dev at 1 m with 1 tag visible. */
  public static double angularStdDevBaseline = 0.06; // radians

  // ── Per-camera trust multipliers ─────────────────────────────────────────
  // Increase a value to trust that camera less; 1.0 = neutral.
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0 — front (wider FOV, better for multi-tag)
        1.0 // Camera 1 — side
      };
}
