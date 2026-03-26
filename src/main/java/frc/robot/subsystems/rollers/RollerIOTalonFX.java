package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX talon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final NeutralOut coastRequest = new NeutralOut();

  public RollerIOTalonFX(int id) {
    talon = new TalonFX(id);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp)
            .isOK();

    inputs.positionRads = position.getValue().in(Radians);
    inputs.velocityRadsPerSec = velocity.getValue().in(RadiansPerSecond);
    inputs.appliedVoltage = appliedVolts.getValue().in(Volts);
    inputs.supplyCurrentAmps = supplyCurrent.getValue().in(Amps);
    inputs.torqueCurrentAmps = torqueCurrent.getValue().in(Amps);
    inputs.tempCelsius = temp.getValue().in(Celsius);

    inputs.hasFollower = false;
    inputs.followerConnected = false;
    inputs.followerSupplyCurrentAmps = 0.0;
    inputs.followerTempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(RollerIOOutputs outputs) {
    switch (outputs.mode) {
      case VOLTAGE_CONTROL -> talon.setControl(voltageRequest.withOutput(outputs.appliedVoltage));
      case COAST -> talon.setControl(coastRequest);
    }
  }
}
