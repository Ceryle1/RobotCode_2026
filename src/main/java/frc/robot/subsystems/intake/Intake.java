package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.rollers.RollerIO;

public class Intake extends SubsystemBase {
  private final Roller roller;

  public Intake(RollerIO io) {
    roller = new Roller("Intake", io);
  }

  @Override
  public void periodic() {
    roller.periodic();
  }

  // API
  public void run() {
    roller.setVoltage(HighAltitudeConstants.Intake.INTAKE_VOLTAGE);
  }

  public void eject() {
    roller.setVoltage(-HighAltitudeConstants.Intake.INTAKE_VOLTAGE);
  }

  public void stop() {
    roller.stop();
  }

  // Getters
  public double getVelocityRadsPerSec() {
    return roller.getVelocityRadsPerSec();
  }

  public boolean isConnected() {
    return roller.isConnected();
  }
}
