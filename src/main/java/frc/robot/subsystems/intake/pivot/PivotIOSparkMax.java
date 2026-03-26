package frc.robot.subsystems.intake.pivot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.HighAltitudeConstants;

public class PivotIOSparkMax implements PivotIO {
  private final SparkMax spark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  public PivotIOSparkMax(int id) {
    spark = new SparkMax(id, MotorType.kBrushless);
    encoder = spark.getEncoder();
    pid = spark.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(40);

    config.encoder.positionConversionFactor(1.0);
    config.inverted(true);

    config
        .closedLoop
        .p(0.8)
        .i(0.0)
        .d(0.005)
        .outputRange(-0.5, 0.5)
        .maxMotion
        .maxVelocity(3000)
        .maxAcceleration(2000)
        .allowedClosedLoopError(0.05);

    config
        .softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(
            (float) HighAltitudeConstants.Pivot.EXTENDED_POS
                * HighAltitudeConstants.Pivot.GEAR_RATIO)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(
            (float) HighAltitudeConstants.Pivot.RETRACT_POS
                * HighAltitudeConstants.Pivot.GEAR_RATIO);

    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRot = encoder.getPosition();
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVoltage = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.supplyCurrentAmps = spark.getOutputCurrent();
    inputs.tempCelsius = spark.getMotorTemperature();
  }

  @Override
  public void applyOutputs(PivotIOOutputs outputs) {
    pid.setReference(
        outputs.targetPositionRot, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0.0);
  }
}
