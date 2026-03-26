package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  @AutoLog
  public static class RollerIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;

    public boolean hasFollower;
    public boolean followerConnected;
    public double followerSupplyCurrentAmps;
    public double followerTempCelsius;
  }

  public enum RollerIOMode {
    COAST,
    VOLTAGE_CONTROL
  }

  public static class RollerIOOutputs {
    public RollerIOMode mode = RollerIOMode.COAST;

    public double appliedVoltage = 0.0;
  }

  default void updateInputs(RollerIOInputs inputs) {}

  default void applyOutputs(RollerIOOutputs outputs) {}
}
