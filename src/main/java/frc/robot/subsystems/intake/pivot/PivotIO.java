package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {
    public double positionRot = 0.0; // motor rotations (raw, pre gear ratio)
    public double velocityRPM = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public static class PivotIOOutputs {
    public double targetPositionRot = 0.0; // motor rotations
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void applyOutputs(PivotIOOutputs outputs) {}

  default void resetEncoder() {}
}
