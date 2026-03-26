package frc.robot.subsystems.rollers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class RollerIOSparkMax implements RollerIO {
  private final SparkMax spark;
  private final RelativeEncoder encoder;

  public RollerIOSparkMax(int id) {
    spark = new SparkMax(id, MotorType.kBrushless);
    encoder = spark.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(40);
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.connected = !spark.hasActiveFault();
    inputs.appliedVoltage = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.supplyCurrentAmps = spark.getOutputCurrent();
    inputs.torqueCurrentAmps = 0.0;
    inputs.tempCelsius = spark.getMotorTemperature();
    inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition());

    inputs.hasFollower = false;
    inputs.followerConnected = false;
    inputs.followerSupplyCurrentAmps = 0.0;
    inputs.followerTempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(RollerIOOutputs outputs) {
    switch (outputs.mode) {
      case VOLTAGE_CONTROL -> spark.setVoltage(outputs.appliedVoltage);
      case COAST -> spark.stopMotor();
    }
  }
}
