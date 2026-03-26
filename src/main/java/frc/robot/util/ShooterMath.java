package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.HighAltitudeConstants.Shooter;

public class ShooterMath {

  public static double getDistanceToGoal(Pose2d robotPose) {
    Translation2d target = FieldConstants.onAlliance(FieldConstants.BLUE_HUB_CENTER);
    double dx = target.getX() - robotPose.getX();
    double dy = target.getY() - robotPose.getY();
    return Math.sqrt(dx * dx + dy * dy);
  }

  public static double calculateHoodAngleRad(Pose2d robotPose) {
    double distance = getDistanceToGoal(robotPose);
    double heightDelta = Shooter.GOAL_HEIGHT - Shooter.SHOOTER_HEIGHT;
    return MathUtil.clamp(
        Math.atan2(heightDelta, distance), Shooter.HOOD_MIN_ANGLE_RAD, Shooter.HOOD_MAX_ANGLE_RAD);
  }
}
