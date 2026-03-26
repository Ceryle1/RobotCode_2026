package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.HighAltitudeConstants;

public class HoodIOSim implements HoodIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1),
          6.0, // Gear Ratio
          0.01, // Moment of Inertia (Estimado)
          0.2, // Longitud (m)
          0.0, // Min Angle (Rad)
          Math.toRadians(45), // Max Angle (Rad)
          true, // Simulate gravity
          0.0 // Starting angle
          );

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    sim.update(HighAltitudeConstants.LOOP_PERIOD_SECS);

    inputs.positionRad = sim.getAngleRads();
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.tempCelsius = new double[] {20.0};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setPosition(double positionRad) {
    // PID simple en simulación para imitar comportamiento
    double feedbackVolts = (positionRad - sim.getAngleRads()) * 20.0;
    setVoltage(feedbackVolts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
