package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionMode;
import frc.robot.util.ShooterMath;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoAimCommand extends Command {

  // Heading PID
  private static final double HEADING_KP = 4.0;
  private static final double HEADING_KI = 0.0;
  private static final double HEADING_KD = 0.3;

  /** Robot is "aimed" when heading error is within this many degrees. */
  private static final double HEADING_TOLERANCE_DEG = 1.5;

  /** Max rotation speed the heading PID can command (rad/s). */
  private static final double MAX_OMEGA_RAD_S = 4.0;

  // ── Dependencies ──────────────────────────────────────────────────────────

  private final SwerveDrive drive;
  private final Hood hood;
  private final Flywheel flywheel;
  private final Vision vision;
  private final Supplier<Double> forwardSupplier;
  private final Supplier<Double> strafeSupplier;

  private final PIDController headingPID;

  /**
   * Aims the robot at the hub using field-geometry heading from odometry pose. Simultaneously spins
   * up the flywheel and sets the hood angle. Driver retains full translation control.
   *
   * @param drive Swerve drivetrain.
   * @param hood Hood subsystem.
   * @param flywheel Flywheel subsystem.
   * @param vision Vision subsystem (will be set to HEADING_ONLY for the duration).
   * @param forwardSupplier Driver forward input, already scaled to m/s.
   * @param strafeSupplier Driver strafe input, already scaled to m/s.
   */
  public AutoAimCommand(
      SwerveDrive drive,
      Hood hood,
      Flywheel flywheel,
      Vision vision,
      Supplier<Double> forward,
      Supplier<Double> strafe) {
    this.drive = drive;
    this.hood = hood;
    this.flywheel = flywheel;
    this.vision = vision;
    this.forwardSupplier = forward;
    this.strafeSupplier = strafe;

    headingPID = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);
    headingPID.enableContinuousInput(-Math.PI, Math.PI);
    headingPID.setTolerance(Units.degreesToRadians(HEADING_TOLERANCE_DEG));

    addRequirements(drive, hood, flywheel, vision);
  }

  @Override
  public void initialize() {
    headingPID.reset();
    // Stop pose estimation while aiming — prevents a sudden vision correction
    // from kicking the odometry mid-shot and corrupting the heading calculation.
    vision.setMode(VisionMode.HEADING_ONLY);
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();

    // ── Target heading from field geometry ───────────────────────────────
    // Uses the alliance-correct hub position from FieldConstants.
    Translation2d target = FieldConstants.onAlliance(FieldConstants.BLUE_HUB_CENTER);
    double dx = target.getX() - pose.getX();
    double dy = target.getY() - pose.getY();
    double targetHeading = Math.atan2(dy, dx);

    // ── Heading PID ──────────────────────────────────────────────────────
    double omega = headingPID.calculate(pose.getRotation().getRadians(), targetHeading);
    // Clamp so the robot can't snap dangerously fast at large initial errors
    omega = MathUtil.clamp(omega, -MAX_OMEGA_RAD_S, MAX_OMEGA_RAD_S);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardSupplier.get() * HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S,
            strafeSupplier.get() * HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S,
            omega,
            pose.getRotation()));

    // ── Shooter ──────────────────────────────────────────────────────────
    // Use ShooterMath for distance-based calculations.
    double distanceM = pose.getTranslation().getDistance(target);
    hood.setAngle(ShooterMath.calculateHoodAngleRad(pose));
    flywheel.runVelocity(Units.rotationsPerMinuteToRadiansPerSecond(4800));

    // ── Logging ──────────────────────────────────────────────────────────
    Logger.recordOutput("AutoAim/TargetHeadingDeg", Units.radiansToDegrees(targetHeading));
    Logger.recordOutput("AutoAim/CurrentHeadingDeg", pose.getRotation().getDegrees());
    Logger.recordOutput("AutoAim/HeadingErrorDeg", Units.radiansToDegrees(headingPID.getError()));
    Logger.recordOutput("AutoAim/OmegaOutput", omega);
    Logger.recordOutput("AutoAim/IsAimed", headingPID.atSetpoint());
    Logger.recordOutput("AutoAim/DistanceM", distanceM);
  }

  @Override
  public void end(boolean interrupted) {
    vision.setMode(VisionMode.DISABLED);

    hood.stop();
    flywheel.coast();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isAimed() {
    return headingPID.atSetpoint();
  }
}
