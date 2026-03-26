package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DefaultSwerveDriveNew extends Command {
  private final SwerveDrive swerve;
  private final DoubleSupplier vXSupplier;
  private final DoubleSupplier vYSupplier;
  private final DoubleSupplier omegaSupplier;
  private final BooleanSupplier precisionModeSupplier;
  private final BooleanSupplier fieldOrientedSupplier;

  /**
   * Comando por defecto para el Swerve Drive optimizado para 2026. Implementa curva exponencial
   * agresiva para los Krakens, compensación de Skew mediante discretización y compatibilidad
   * estricta con AdvantageKit (Replay-safe).
   */
  public DefaultSwerveDriveNew(
      SwerveDrive swerve,
      DoubleSupplier vXSupplier,
      DoubleSupplier vYSupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier precisionModeSupplier,
      BooleanSupplier fieldOrientedSupplier) {
    this.swerve = swerve;
    this.vXSupplier = vXSupplier;
    this.vYSupplier = vYSupplier;
    this.omegaSupplier = omegaSupplier;
    this.precisionModeSupplier = precisionModeSupplier;
    this.fieldOrientedSupplier = fieldOrientedSupplier;

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    // 1. Leer valores crudos
    double vX = vXSupplier.getAsDouble();
    double vY = vYSupplier.getAsDouble();
    double omega = omegaSupplier.getAsDouble();
    boolean isPrecisionMode = precisionModeSupplier.getAsBoolean();
    boolean isFieldOriented = fieldOrientedSupplier.getAsBoolean();

    // 2. Deadband Centralizado
    vX = MathUtil.applyDeadband(vX, 0.08);
    vY = MathUtil.applyDeadband(vY, 0.08);
    omega = MathUtil.applyDeadband(omega, 0.08);

    // 3. Curva Exponencial (SQUARING ÚNICO)
    vX = Math.copySign(vX * vX, vX);
    vY = Math.copySign(vY * vY, vY);
    omega = Math.copySign(omega * omega, omega);

    // 4. Multiplicadores de Modo (Precisión vs Sprints)
    double transMultiplier = isPrecisionMode ? 0.30 : 1.0; // Cambiado a 1.0 para sprint máximo
    double rotMultiplier = isPrecisionMode ? 0.50 : 0.85; // Subido a 0.85 para que SÍ gire ágil

    vX *= transMultiplier * HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S;
    vY *= transMultiplier * HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S;
    omega *= rotMultiplier * HighAltitudeConstants.Swerve.MAX_ANGULAR_SPEED_RAD_S;

    // 5. Transformación Field-Oriented con ALLIANCE FLIP 🔴🔵
    ChassisSpeeds desiredSpeeds;
    if (isFieldOriented) {
      // Magia de Arquitectura: Si somos alianza roja, invertimos traslación para que
      // se maneje igual.
      var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
      if (alliance.isPresent()
          && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
        vX = -vX;
        vY = -vY;
      }

      desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, omega, swerve.getRotation());
    } else {
      desiredSpeeds = new ChassisSpeeds(vX, vY, omega);
    }

    // 6. Skew Compensation (Discretización)
    ChassisSpeeds discretSpeeds = ChassisSpeeds.discretize(desiredSpeeds, 0.02);

    // 7. Enviar comandos
    swerve.runVelocity(discretSpeeds);

    // 8. AdvantageKit Logging
    Logger.recordOutput("Swerve/Commands/DefaultDrive/IsPrecisionMode", isPrecisionMode);
    Logger.recordOutput(
        "Swerve/Commands/DefaultDrive/vxMetersPerSecond", discretSpeeds.vxMetersPerSecond);
    Logger.recordOutput(
        "Swerve/Commands/DefaultDrive/vyMetersPerSecond", discretSpeeds.vyMetersPerSecond);
    Logger.recordOutput(
        "Swerve/Commands/DefaultDrive/omegaRadiansPerSecond", discretSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    // Enviar ceros absolutos al interrumpirse o cancelar para frenar de inmediato
    swerve.runVelocity(new ChassisSpeeds());
  }
}
