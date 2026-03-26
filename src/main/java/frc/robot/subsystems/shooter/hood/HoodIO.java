package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double positionRad = 0.0;
    public double targetPositionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  /** Actualiza el conjunto de inputs desde el hardware/simulación. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Envía control de posición (PID onboard). */
  public default void setPosition(double positionRad) {}

  /** Envía control de voltaje manual. */
  public default void setVoltage(double volts) {}

  /** Detiene el motor. */
  public default void stop() {}
}
