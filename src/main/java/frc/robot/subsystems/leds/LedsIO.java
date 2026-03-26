package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LedsIO {
  @AutoLog
  public static class LedsIOInputs {}

  public static class LedsIOOutputs {
    public byte[] buffer = new byte[0];
  }

  default void updateInputs(LedsIOInputs inputs) {}

  default void applyOutputs(LedsIOOutputs outputs) {}
}
