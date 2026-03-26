package frc.robot.subsystems.swerve.gyro;

public class GyroIOSim implements GyroIO {
  private double yawRad = 0.0;
  private double yawVelocityRadPerSec = 0.0;

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;

    // Integrar la velocidad angular para simular la posición (20ms por loop)
    yawRad += yawVelocityRadPerSec * 0.02;

    inputs.yawPositionRad = yawRad;
    inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
  }

  // NUEVO: Método para que el SwerveDrive le inyecte la velocidad
  public void setYawData(double velocityRadPerSec) {
    this.yawVelocityRadPerSec = velocityRadPerSec;
  }
}
