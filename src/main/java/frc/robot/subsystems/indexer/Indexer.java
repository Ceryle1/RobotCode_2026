package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.rollers.RollerIO;

public class Indexer extends SubsystemBase {
  private final Roller roller;

  public Indexer(RollerIO io) {
    roller = new Roller("Indexer", io);
  }

  @Override
  public void periodic() {
    roller.periodic();
  }

  // API
  public void intake() {
    roller.setVoltage(HighAltitudeConstants.Indexer.INDEXER_VOLTAGE);
  }

  public void eject() {
    roller.setVoltage(-HighAltitudeConstants.Indexer.INDEXER_VOLTAGE);
  }

  public void stop() {
    roller.stop();
  }

  // Getters
  public double getVelocityRadsPerSec() {
    return roller.getVelocityRadsPerSec();
  }

  public boolean isStalled() {
    return roller.getCurrentAmps() > HighAltitudeConstants.Indexer.STALL_CURRENT_AMPS;
  }

  public boolean isConnected() {
    return roller.isConnected();
  }
}
