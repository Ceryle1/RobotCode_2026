package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants.Auto;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Teleop Assist: Permite al driver controlar la traslación (X/Y) libremente mientras el robot
 * mantiene automáticamente el frente apuntando a un objetivo (Hub/Feeder). Soporta Live Tuning
 * usando las constantes de Travel Mode.
 */
public class SwerveHeadingLockCommand extends Command {
  private final SwerveDrive drive;
  private final DoubleSupplier xRequest;
  private final DoubleSupplier yRequest;
  private final Supplier<Translation2d> targetPointSupplier;

  private final ProfiledPIDController thetaController;

  public SwerveHeadingLockCommand(
      SwerveDrive drive,
      DoubleSupplier xRequest,
      DoubleSupplier yRequest,
      Supplier<Translation2d> targetPointSupplier) {

    this.drive = drive;
    this.xRequest = xRequest;
    this.yRequest = yRequest;
    this.targetPointSupplier = targetPointSupplier;

    thetaController =
        new ProfiledPIDController(
            Auto.travelSteerKp.get(),
            Auto.travelSteerKi.get(),
            Auto.travelSteerKd.get(),
            new TrapezoidProfile.Constraints(
                Auto.TRAVEL_ANGULAR_VELOCITY, Auto.TRAVEL_ANGULAR_ACCELERATION));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Auto.POSE_TOLERANCE_RADIANS);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reseteamos el PID al estado actual para evitar saltos
    thetaController.reset(
        drive.getRotation().getRadians(), drive.getFieldRelativeSpeeds().omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    // --- LIVE TUNING LOGIC ---
    // Permite ajustar la agresividad del "Snap" en tiempo real desde AdvantageScope
    if (Auto.travelSteerKp.hasChanged(thetaController.getP()))
      thetaController.setP(Auto.travelSteerKp.get());
    if (Auto.travelSteerKi.hasChanged(thetaController.getI()))
      thetaController.setI(Auto.travelSteerKi.get());
    if (Auto.travelSteerKd.hasChanged(thetaController.getD()))
      thetaController.setD(Auto.travelSteerKd.get());

    // --- LÓGICA DE CONTROL ---
    Pose2d currentPose = drive.getPose();
    Translation2d targetPoint = targetPointSupplier.get();

    // 1. Calcular Ángulo Deseado
    double targetAngleRad =
        Math.atan2(
            targetPoint.getY() - currentPose.getY(), targetPoint.getX() - currentPose.getX());

    // 2. Calcular Velocidad Rotacional (PID)
    double rotationOutput =
        thetaController.calculate(currentPose.getRotation().getRadians(), targetAngleRad);

    // 3. Procesar Inputs del Driver
    // Multiplicamos por la velocidad máxima definida en constantes
    double xVelocity = xRequest.getAsDouble() * Auto.TRAVEL_LINEAR_VELOCITY;
    double yVelocity = yRequest.getAsDouble() * Auto.TRAVEL_LINEAR_VELOCITY;

    // 4. Combinar y Enviar
    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationOutput);

    // Convertimos a Robot Relative para el Swerve
    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentPose.getRotation());

    drive.runVelocity(robotRelativeSpeeds);

    // Logging
    Logger.recordOutput("HeadingLock/TargetPoint", targetPoint);
    Logger.recordOutput("HeadingLock/TargetAngle", Rotation2d.fromRadians(targetAngleRad));
    Logger.recordOutput("HeadingLock/ErrorDeg", Math.toDegrees(thetaController.getPositionError()));
  }

  @Override
  public void end(boolean interrupted) {
    // Transición suave al DefaultCommand
  }
}
