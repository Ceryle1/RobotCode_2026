package frc.robot.commands.swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.Swerve;
import frc.robot.controls.ControlProfile;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;

public class SwerveDefaultCommand extends Command {
  private final SwerveDrive drive;
  private final ControlProfile controller;
  private final BooleanSupplier isFieldOriented;

  // Slew Rate Limiters (Filtros para suavizar el input del joystick)
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(Swerve.MAX_LINEAR_SPEED_M_S);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(Swerve.MAX_LINEAR_SPEED_M_S);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(Swerve.TURN_MAX_VELOCITY_RAD_S);

  @Deprecated
  public SwerveDefaultCommand(
      SwerveDrive drive, ControlProfile controller, BooleanSupplier isFieldOriented) {
    this.drive = drive;
    this.controller = controller;
    this.isFieldOriented = isFieldOriented;
    addRequirements(drive);
  }

  // Constructor conveniente que asume Field Oriented true
  public SwerveDefaultCommand(SwerveDrive drive, ControlProfile controller) {
    this(drive, controller, () -> true);
  }

  @Override
  public void execute() {
    // 1. Obtener Inputs
    double xInput = controller.getDriveForward();
    double yInput = controller.getDriveStrafe();
    double rotInput = controller.getDriveRotation();

    // 2. Aplicar Slew Rate (Suavizado)
    xInput = xLimiter.calculate(xInput);
    yInput = yLimiter.calculate(yInput);
    rotInput = rotLimiter.calculate(rotInput);

    // 3. Escalar a Metros/Segundos reales
    double xMetersPerSec = xInput * HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S;
    double yMetersPerSec = yInput * HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S;
    double rotRadPerSec = rotInput * HighAltitudeConstants.Swerve.MAX_ANGULAR_SPEED_RAD_S;

    // 4. Calcular ChassisSpeeds
    ChassisSpeeds speeds;
    if (isFieldOriented.getAsBoolean()) {
      var alliance = DriverStation.getAlliance();

      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                -xMetersPerSec, -yMetersPerSec, rotRadPerSec, drive.getRotation());

      } else {

        speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xMetersPerSec, yMetersPerSec, rotRadPerSec, drive.getRotation());
      }
    } else {

      // Robot Oriented
      speeds = new ChassisSpeeds(xMetersPerSec, yMetersPerSec, rotRadPerSec);
    }

    // 6. Enviar al Drive
    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
