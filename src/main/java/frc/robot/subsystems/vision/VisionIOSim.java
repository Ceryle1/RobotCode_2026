package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionIOSim implements VisionIO {
  private final Supplier<Pose2d> poseSupplier;

  public VisionIOSim(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = true;
    inputs.latestTargetObservation =
        new TargetObservation(
            new edu.wpi.first.math.geometry.Rotation2d(),
            new edu.wpi.first.math.geometry.Rotation2d());

    Pose2d currentPose = poseSupplier.get();
    Pose3d noisyPose =
        new Pose3d(
            currentPose.getX() + (Math.random() - 0.5) * 0.02,
            currentPose.getY() + (Math.random() - 0.5) * 0.02,
            0.0,
            new Rotation3d(currentPose.getRotation()));

    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              Logger.getTimestamp() / 1_000_000.0,
              noisyPose,
              0.05,
              2,
              2.0,
              PoseObservationType.PHOTONVISION_MULTITAG)
        };

    inputs.tagIds = new int[] {1, 2};
  }
}
