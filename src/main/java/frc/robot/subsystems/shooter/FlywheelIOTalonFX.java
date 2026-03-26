package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.HighAltitudeConstants;

public class FlywheelIOTalonFX implements FlywheelIO {

  // Hardware
  private final TalonFX leader;
  private final TalonFX follower;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemp;

  // Control Requests
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut coastRequest = new NeutralOut();

  public FlywheelIOTalonFX(int leaderID, int followerID) {
    leader = new TalonFX(leaderID);
    follower = new TalonFX(followerID);

    // --- Leader config ---
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    leaderConfig.Slot0.kP = HighAltitudeConstants.Shooter.KP;
    leaderConfig.Slot0.kD = HighAltitudeConstants.Shooter.KD;
    leaderConfig.Slot0.kV = HighAltitudeConstants.Shooter.KV;
    leader.getConfigurator().apply(leaderConfig);

    // --- Follower config ---
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    followerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    followerConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    follower.getConfigurator().apply(followerConfig);

    // Opposite direction to leader
    follower.setControl(new Follower(leaderID, MotorAlignmentValue.Opposed));

    // --- Signal Setup ---
    position = leader.getPosition();
    velocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    supplyCurrent = leader.getSupplyCurrent();
    torqueCurrent = leader.getTorqueCurrent();
    temp = leader.getDeviceTemp();
    followerSupplyCurrent = follower.getSupplyCurrent();
    followerTemp = follower.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        20.0,
        position,
        velocity,
        appliedVolts,
        supplyCurrent,
        torqueCurrent,
        temp,
        followerSupplyCurrent,
        followerTemp);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, supplyCurrent, torqueCurrent, temp)
            .isOK();
    inputs.followerConnected =
        BaseStatusSignal.refreshAll(followerSupplyCurrent, followerTemp).isOK();

    inputs.positionRads = position.getValue().in(Radians);
    inputs.velocityRadsPerSec = velocity.getValue().in(RadiansPerSecond);
    inputs.appliedVoltage = appliedVolts.getValue().in(Volts);
    inputs.supplyCurrentAmps = supplyCurrent.getValue().in(Amps);
    inputs.torqueCurrentAmps = torqueCurrent.getValue().in(Amps);
    inputs.tempCelsius = temp.getValue().in(Celsius);
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValue().in(Amps);
    inputs.followerTempCelsius = followerTemp.getValue().in(Celsius);
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode) {
      case VELOCITY -> leader.setControl(
          velocityRequest.withVelocity(outputs.velocityRadsPerSec / (2.0 * Math.PI)));
      case COAST -> leader.setControl(coastRequest);
    }
  }
}
