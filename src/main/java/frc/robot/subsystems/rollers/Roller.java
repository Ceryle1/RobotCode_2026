package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerIO.RollerIOOutputs;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  private final RollerIOOutputs outputs = new RollerIOOutputs();
  private final String name;

  public Roller(String name, RollerIO io) {
    this.name = name;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    io.applyOutputs(outputs);
    Logger.recordOutput(name + "/Mode", outputs.mode.toString());
    Logger.recordOutput(name + "/Voltage", outputs.appliedVoltage);
  }

  // API
  public void setVoltage(double voltage) {
    outputs.mode = RollerIO.RollerIOMode.VOLTAGE_CONTROL;
    outputs.appliedVoltage = voltage;
  }

  public void stop() {
    outputs.mode = RollerIO.RollerIOMode.COAST;
    outputs.appliedVoltage = 0.0;
  }

  // Getters
  public double getVelocityRadsPerSec() {
    return inputs.velocityRadsPerSec;
  }

  public double getCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public boolean isConnected() {
    return inputs.connected;
  }
}
