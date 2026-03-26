package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.HighAltitudeConstants;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX motor;

  // Relación
  private static final double GEAR_RATIO = 42.5;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;

  private final PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);
  private final VoltageOut voltageControl = new VoltageOut(0);

  public HoodIOTalonFX(int motorId) {
    motor = new TalonFX(motorId);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Configuración PID Onboard (Valores de ejemplo)
    config.Slot0.kP = HighAltitudeConstants.Shooter.HOOD_kP;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = HighAltitudeConstants.Shooter.HOOD_kD;

    // Feedback con el ratio aplicado
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    motor.getConfigurator().apply(config);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent, temp);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, supplyCurrent, temp);

    inputs.positionRad = position.getValue().in(Radians);
    inputs.velocityRadPerSec = velocity.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = appliedVolts.getValue().in(Volts);
    inputs.currentAmps = new double[] {supplyCurrent.getValue().in(Amps)};
    inputs.tempCelsius = new double[] {temp.getValue().in(Celsius)};
  }

  @Override
  public void setPosition(double positionRad) {
    // El motor ya tiene el ratio configurado, enviamos rotaciones directamente
    motor.setControl(positionControl.withPosition(positionRad / (2.0 * Math.PI)));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.setControl(voltageControl.withOutput(0));
  }
}
