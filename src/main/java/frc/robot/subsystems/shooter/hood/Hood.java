package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants.Shooter;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  /** Inyección de dependencias para cumplir con la arquitectura HAL. */
  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  /** Mueve el Hood a un ángulo específico. */
  public void setAngle(double angleRad) {

    double clampedAngle =
        MathUtil.clamp(angleRad, Shooter.HOOD_MIN_ANGLE_RAD, Shooter.HOOD_MAX_ANGLE_RAD);
    io.setPosition(clampedAngle);
    Logger.recordOutput("Hood/TargetPositionRad", clampedAngle);
  }

  /** Control manual por voltaje (ej. para caracterización o recuperación). */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void stop() {
    io.stop();
  }

  /** Retorna la posición actual para lógica de AutoAim o Fin de Comando. */
  public double getPositionRad() {
    return inputs.positionRad;
  }
}
