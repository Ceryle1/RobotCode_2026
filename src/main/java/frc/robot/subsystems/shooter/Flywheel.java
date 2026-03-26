package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.FlywheelIO.FlywheelIOOutputs;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    io.applyOutputs(outputs);
    Logger.recordOutput("Flywheel/Mode", outputs.mode.toString());
    Logger.recordOutput("Flywheel/TargetVelocityRadsPerSec", outputs.velocityRadsPerSec);
  }

  // API

  public void runVelocity(double velocityRadsPerSec) {
    outputs.mode = FlywheelIO.FlywheelIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
  }

  public void coast() {
    outputs.mode = FlywheelIO.FlywheelIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
  }

  // Getters

  public double getVelocityRadsPerSec() {
    return inputs.velocityRadsPerSec;
  }

  public double getPositionRads() {
    return inputs.positionRads;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public boolean atSetpoint(double toleranceRadsPerSec) {
    return Math.abs(inputs.velocityRadsPerSec - outputs.velocityRadsPerSec) < toleranceRadsPerSec;
  }
}
