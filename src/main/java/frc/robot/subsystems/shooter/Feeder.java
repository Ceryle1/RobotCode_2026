package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.rollers.RollerIO;

public class Feeder extends SubsystemBase {
  private final Roller roller;

  public Feeder(RollerIO io) {
    roller = new Roller("Feeder", io);
  }

  @Override
  public void periodic() {
    roller.periodic();
  }

  // API
  public void intake() {
    roller.setVoltage(HighAltitudeConstants.Feeder.FEEDER_VOLTAGE);
  }

  public void eject() {
    roller.setVoltage(-HighAltitudeConstants.Feeder.FEEDER_VOLTAGE);
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
