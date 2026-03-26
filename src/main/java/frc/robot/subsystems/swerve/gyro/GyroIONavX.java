package frc.robot.subsystems.swerve.gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;

public class GyroIONavX implements GyroIO {
  private final AHRS navx;

  public GyroIONavX() {
    navx = new AHRS(NavXComType.kMXP_SPI);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navx.isConnected();

    // TASK 1: Guardia de seguridad durante calibración
    if (navx.isCalibrating()) {
      return;
    }

    // NavX is CW positive by default, WPILib/AK expects CCW positive (NWU).
    // inputs.yawPositionRad = Units.degreesToRadians(navx.getAngle());
    inputs.yawPositionRad =
        Units.degreesToRadians(navx.getAngle()); // + 0.886; // +offset deg to rad aprox 0.433 rad
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(navx.getRate());
  }

  @Override
  public void reset() {
    navx.reset();
  }
}
