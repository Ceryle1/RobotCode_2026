package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * VisionIO implementation for the Meta Quest (QuestNav).
 *
 * <p>QuestNav publishes its pose over NetworkTables. The Quest operates in its own coordinate
 * frame, so we apply a one-time origin reset on first connect to align it with the WPILib field
 * coordinate system.
 *
 * <p>Wire-up in RobotContainer (REAL case):
 *
 * <pre>
 * vision = new Vision(
 *     drive::addVisionMeasurement,
 *     new VisionIOPhotonVision(VisionConstants.CAMERA_FRONT_NAME, VisionConstants.robotToCamera0),
 *     new VisionIOPhotonVision(VisionConstants.CAMERA_BACK_NAME,  VisionConstants.robotToCamera1),
 *     new VisionIOQuestNav());
 * </pre>
 *
 * <p>Also add a corresponding entry to {@link VisionConstants#cameraStdDevFactors} for this index.
 */
public class VisionIOQuestNav implements VisionIO {

  // ── NetworkTables topics published by the Quest companion app ──────────────
  private static final String NT_TABLE = "questnav";

  private final NetworkTable table;
  private final DoubleArraySubscriber poseSub; // [x, y, z, qw, qx, qy, qz]
  private final IntegerSubscriber frameSub; // monotonically-increasing frame counter
  private final DoubleArraySubscriber timestampSub; // [seconds] — same timebase as WPILib

  // Used to detect a newly-connected or reset Quest so we can re-zero the origin
  private long lastFrameCount = -1;
  private boolean originSet = false;

  public VisionIOQuestNav() {
    table = NetworkTableInstance.getDefault().getTable(NT_TABLE);
    poseSub = table.getDoubleArrayTopic("pose").subscribe(new double[0]);
    frameSub = table.getIntegerTopic("frameCount").subscribe(-1);
    timestampSub = table.getDoubleArrayTopic("timestamp").subscribe(new double[0]);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    long frame = frameSub.get();
    inputs.connected = (frame != -1);

    if (!inputs.connected) {
      originSet = false; // Reset so we re-zero on reconnect
      inputs.poseObservations = new PoseObservation[0];
      inputs.tagIds = new int[0];
      return;
    }

    double[] poseData = poseSub.get();
    double[] timestampData = timestampSub.get();

    // Need at least a 7-element pose and a timestamp
    if (poseData.length < 7 || timestampData.length < 1) {
      inputs.poseObservations = new PoseObservation[0];
      inputs.tagIds = new int[0];
      return;
    }

    // Only emit a new observation when the Quest publishes a new frame
    if (frame == lastFrameCount) {
      inputs.poseObservations = new PoseObservation[0];
      inputs.tagIds = new int[0];
      return;
    }
    lastFrameCount = frame;

    // ── Build the Pose3d from the Quest quaternion pose ────────────────────
    // Quest coordinate frame: +X right, +Y up, +Z backward (OpenXR / Unity)
    // WPILib field frame:     +X forward, +Y left, +Z up
    //
    // TODO: Verify the exact axis mapping for your Quest mounting orientation.
    //       The transform below is a common starting point — tune it on the
    //       actual robot by driving a known path and comparing to odometry.
    double qx = poseData[0];
    double qy = poseData[1];
    double qz = poseData[2];
    double qw = poseData[3];
    double px = poseData[4];
    double py = poseData[5];
    double pz = poseData[6];

    Pose3d rawPose =
        new Pose3d(
            px, py, pz, new Rotation3d(new edu.wpi.first.math.geometry.Quaternion(qw, qx, qy, qz)));

    double timestamp = timestampData[0];

    // QuestNav provides absolute pose with high confidence — no AprilTag count.
    // We pass tagCount=1 and a fixed "distance" so Vision.java's std dev formula
    // produces reasonable values (controlled via cameraStdDevFactors instead).
    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              timestamp,
              rawPose,
              0.0, // QuestNav doesn't have pose ambiguity
              1, // Not tag-based — set to 1 so stdDev formula doesn't divide by 0
              1.0, // Nominal "distance" — tune via cameraStdDevFactors
              PoseObservationType.QUESTNAV)
        };

    // QuestNav doesn't see AprilTags directly
    inputs.tagIds = new int[0];
  }
}
