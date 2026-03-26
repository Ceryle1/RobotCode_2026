package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected;
    public boolean followerConnected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
    public double followerSupplyCurrentAmps;
    public double followerTempCelsius;
  }

  public static enum FlywheelIOOutputMode {
    COAST,
    VELOCITY
  }

  public static class FlywheelIOOutputs {
    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;
    public double velocityRadsPerSec = 0.0;
    // kP, kD, feedforward removed — gains live in constructor config only
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}
}
