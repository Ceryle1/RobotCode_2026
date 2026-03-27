package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
    double x = getDistanceToGoal(robotPose);
    double y = Shooter.GOAL_HEIGHT - Shooter.SHOOTER_HEIGHT;

    double g = 9.81;

    double omega = Units.rotationsPerMinuteToRadiansPerSecond(3900);
    double v = omega * Shooter.FLYWHEEL_RADIUS * Shooter.SLIP_FACTOR;

    double v2 = v * v;
    double insideSqrt = v2 * v2 - g * (g * x * x + 2 * y * v2);

    if (insideSqrt < 0) {
      return Shooter.HOOD_MAX_ANGLE_RAD;
    }

    double sqrt = Math.sqrt(insideSqrt);

    double theta = Math.atan((v2 - sqrt) / (g * x));

    return MathUtil.clamp(theta, Shooter.HOOD_MIN_ANGLE_RAD, Shooter.HOOD_MAX_ANGLE_RAD);
  }
}
