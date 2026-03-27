package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.ShooterMath;
import java.util.function.Supplier;

public class AutoAimCommand extends Command {
  private final SwerveDrive drive;
  private final Hood hood;
  private final Flywheel flywheel;
  private final Supplier<Double> forwardSupplier;
  private final Supplier<Double> strafeSupplier;

  private final PIDController headingPID =
      new PIDController(HighAltitudeConstants.Auto.PATHPLANNER_ROTATION_KP, 0, HighAltitudeConstants.Auto.PATHPLANNER_ROTATION_KD);

  public AutoAimCommand(
      SwerveDrive drive,
      Hood hood,
      Flywheel flywheel,
      Supplier<Double> forward,
      Supplier<Double> strafe) {
    this.drive = drive;
    this.hood = hood;
    this.flywheel = flywheel;
    this.forwardSupplier = forward;
    this.strafeSupplier = strafe;
    addRequirements(drive, hood, flywheel);
    headingPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    Translation2d target = FieldConstants.onAlliance(FieldConstants.BLUE_HUB_CENTER);

    // Angle from robot to hub
    double dx = target.getX() - pose.getX();
    double dy = target.getY() - pose.getY();
    double targetHeading = Math.atan2(dy, dx);

    double omega = headingPID.calculate(pose.getRotation().getRadians(), targetHeading);

    // Driver can still strafe/drive while locked to hub
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardSupplier.get() * HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S,
            strafeSupplier.get() * HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S,
            omega,
            pose.getRotation()));

    hood.setAngle(ShooterMath.calculateHoodAngleRad(pose));
    flywheel.runVelocity(Units.rotationsPerMinuteToRadiansPerSecond(4800));
  }

  @Override
  public void end(boolean interrupted) {
    hood.stop();
    flywheel.coast();
    drive.stop();
  }
}
