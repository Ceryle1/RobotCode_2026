package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants.Auto;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Comando DriveToPose Replay-Ready. Soporta Dual-Gains (Travel vs Precision) y Live Tuning desde
 * Dashboard.
 */
public class DriveToPose extends Command {
  private final SwerveDrive drive;
  private final Supplier<Pose2d> targetPoseSupplier;

  @SuppressWarnings("unused")
  private final boolean precisionMode;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  // Referencias a los Tunables que usaremos según el modo
  private final LoggedTunableNumber driveKp, driveKi, driveKd;
  private final LoggedTunableNumber steerKp, steerKi, steerKd;

  private Pose2d targetPose;

  public DriveToPose(
      SwerveDrive drive, Supplier<Pose2d> targetPoseSupplier, boolean precisionMode) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    this.precisionMode = precisionMode;

    // 1. Asignar Constantes según el modo
    if (precisionMode) {
      driveKp = Auto.precisionDriveKp;
      driveKi = Auto.precisionDriveKi;
      driveKd = Auto.precisionDriveKd;
      steerKp = Auto.precisionSteerKp;
      steerKi = Auto.precisionSteerKi;
      steerKd = Auto.precisionSteerKd;
    } else {
      driveKp = Auto.travelDriveKp;
      driveKi = Auto.travelDriveKi;
      driveKd = Auto.travelDriveKd;
      steerKp = Auto.travelSteerKp;
      steerKi = Auto.travelSteerKi;
      steerKd = Auto.travelSteerKd;
    }

    // 2. Configurar Constraints (Valores estáticos por seguridad)
    double maxLinVel = precisionMode ? Auto.PRECISION_LINEAR_VELOCITY : Auto.TRAVEL_LINEAR_VELOCITY;

    double maxLinAcc =
        precisionMode ? Auto.PRECISION_LINEAR_ACCELERATION : Auto.TRAVEL_LINEAR_ACCELERATION;

    double maxAngVel =
        precisionMode ? Auto.PRECISION_ANGULAR_VELOCITY : Auto.TRAVEL_ANGULAR_VELOCITY;

    double maxAngAcc =
        precisionMode ? Auto.PRECISION_ANGULAR_ACCELERATION : Auto.TRAVEL_ANGULAR_ACCELERATION;

    // 3. Inicializar Controladores con los valores actuales de los Tunables
    xController =
        new ProfiledPIDController(
            driveKp.get(),
            driveKi.get(),
            driveKd.get(),
            new TrapezoidProfile.Constraints(maxLinVel, maxLinAcc));

    yController =
        new ProfiledPIDController(
            driveKp.get(),
            driveKi.get(),
            driveKd.get(),
            new TrapezoidProfile.Constraints(maxLinVel, maxLinAcc));

    thetaController =
        new ProfiledPIDController(
            steerKp.get(),
            steerKi.get(),
            steerKd.get(),
            new TrapezoidProfile.Constraints(maxAngVel, maxAngAcc));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(Auto.POSE_TOLERANCE_METERS);
    yController.setTolerance(Auto.POSE_TOLERANCE_METERS);
    thetaController.setTolerance(Auto.POSE_TOLERANCE_RADIANS);

    addRequirements(drive);
  }

  public DriveToPose(SwerveDrive drive, Supplier<Pose2d> targetPoseSupplier) {
    this(drive, targetPoseSupplier, false); // Default a Travel Mode
  }

  @Override
  public void initialize() {
    targetPose = targetPoseSupplier.get();
    Pose2d currentPose = drive.getPose();
    ChassisSpeeds currentSpeeds = drive.getFieldRelativeSpeeds();

    // Resetear controladores para iniciar desde el movimiento actual (Motion
    // Profiling continuo)
    xController.reset(currentPose.getX(), currentSpeeds.vxMetersPerSecond);
    yController.reset(currentPose.getY(), currentSpeeds.vyMetersPerSecond);
    thetaController.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    // --- LIVE TUNING LOGIC ---
    // Actualizamos los PIDs si los cambió en el Dashboard
    if (driveKp.hasChanged(xController.getP())) xController.setP(driveKp.get());
    if (driveKi.hasChanged(xController.getI())) xController.setI(driveKi.get());
    if (driveKd.hasChanged(xController.getD())) xController.setD(driveKd.get());

    // Copiamos los valores de X a Y (asumimos simetría en traslación)
    if (yController.getP() != xController.getP()) yController.setP(xController.getP());
    if (yController.getI() != xController.getI()) yController.setI(xController.getI());
    if (yController.getD() != xController.getD()) yController.setD(xController.getD());

    if (steerKp.hasChanged(thetaController.getP())) thetaController.setP(steerKp.get());
    if (steerKi.hasChanged(thetaController.getI())) thetaController.setI(steerKi.get());
    if (steerKd.hasChanged(thetaController.getD())) thetaController.setD(steerKd.get());

    // --- CONTROL LOOP ---
    targetPose = targetPoseSupplier.get();
    Pose2d currentPose = drive.getPose();

    // 1. Calcular Feedback (PID)
    double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
    double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());
    double thetaFeedback =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // 2. Calcular Feedforward (Profile Velocity)
    double xFF = xController.getSetpoint().velocity;
    double yFF = yController.getSetpoint().velocity;
    double thetaFF = thetaController.getSetpoint().velocity;

    // 3. Output (Field Relative -> Robot Relative)
    // Convertimos a Robot Relative para enviar al Swerve
    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFeedback + xFF, yFeedback + yFF, thetaFeedback + thetaFF, currentPose.getRotation());

    drive.runVelocity(robotRelativeSpeeds);

    // --- LOGGING ---
    Logger.recordOutput("DriveToPose/TargetPose", targetPose);
    Logger.recordOutput(
        "DriveToPose/SetpointPose",
        new Pose2d(
            xController.getSetpoint().position,
            yController.getSetpoint().position,
            new Rotation2d(thetaController.getSetpoint().position)));

    // Errores para graficar en AdvantageScope
    Logger.recordOutput("DriveToPose/Error/X_Meters", xController.getPositionError());
    Logger.recordOutput("DriveToPose/Error/Y_Meters", yController.getPositionError());
    Logger.recordOutput(
        "DriveToPose/Error/Theta_Deg", Units.radiansToDegrees(thetaController.getPositionError()));
  }

  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
