package frc.robot.controls;

import frc.robot.RobotContainer;

public interface ControlProfile {
  /** Retorna la velocidad frontal deseada (-1.0 a 1.0). Positivo = Hacia adelante. */
  double getDriveForward();

  /**
   * Retorna la velocidad lateral deseada (-1.0 a 1.0). Positivo = Hacia la izquierda (Estándar
   * WPILib NWU).
   */
  double getDriveStrafe();

  /**
   * Retorna la velocidad de rotación deseada (-1.0 a 1.0). Positivo = Sentido Antihorario (CCW).
   */
  double getDriveRotation();

  /**
   * Retorna la velocidad del flywheel deseada (0.0 a 1.0). Default 0.0 para perfiles que no usan el
   * shooter.
   */
  default double getShooterTrigger() {
    return 0.0;
  }

  default boolean getPivotExpand() {
    return false;
  }

  default boolean getPivotRetract() {
    return false;
  }

  /**
   * Configura los botones específicos de este perfil.
   *
   * @param container Referencia al contenedor para acceder a los subsistemas.
   */
  void configureBindings(RobotContainer container);
}
