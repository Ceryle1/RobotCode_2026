package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HighAltitudeConstants;

/**
 * Clase utilitaria para valores que pueden ajustarse en tiempo real (Live Tuning). *
 * Comportamiento: - Si HighAltitudeConstants.TUNING_MODE es true: Publica en SmartDashboard/NT y
 * lee cambios dinámicamente. - Si es false: Usa siempre el valor default (seguridad para matches).
 * * Ideal para PIDs que necesitan ajustarse en Replay.
 */
public class LoggedTunableNumber {
  private final String key;
  private final double defaultValue;
  private double lastValue;
  private boolean hasDefaulted = false;

  /**
   * Crea un nuevo número tuneable.
   *
   * @param dashboardKey La ruta en NetworkTables (ej. "Tuning/Drive/kP")
   * @param defaultValue El valor por defecto (Hardcoded)
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this.key = dashboardKey;
    this.defaultValue = defaultValue;
    this.lastValue = defaultValue;
  }

  /**
   * Obtiene el valor actual. Lee de NetworkTables si estamos en Tuning Mode, sino devuelve el
   * default.
   */
  public double get() {
    if (!HighAltitudeConstants.TUNING_MODE) {
      return defaultValue;
    }

    if (!hasDefaulted) {
      // Inicializar el valor en el Dashboard si no existe
      if (!SmartDashboard.containsKey(key)) {
        SmartDashboard.putNumber(key, defaultValue);
      }
      hasDefaulted = true;
    }

    // Leer el valor actual (usuario pudo haberlo cambiado)
    double currentValue = SmartDashboard.getNumber(key, defaultValue);
    lastValue = currentValue;
    return currentValue;
  }

  /**
   * Verifica si el valor ha cambiado desde la última vez que se revisó. Útil para no spammear
   * actualizaciones de PID en cada ciclo.
   */
  public boolean hasChanged(double currentValue) {
    return currentValue != lastValue;
  }

  /** Verifica si el valor ha cambiado respecto a una referencia externa. */
  public boolean hasChanged(double id, double currentValue) {
    // Implementación simplificada para uso general
    return get() != id;
  }
}
