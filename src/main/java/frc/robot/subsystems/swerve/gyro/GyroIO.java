package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double yawPositionRad = 0.0;
    public double yawVelocityRadPerSec = 0.0;
  }

  default void updateInputs(GyroIOInputs inputs) {}

  default void reset() {}
}
