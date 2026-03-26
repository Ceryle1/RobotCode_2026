package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.util.PhoenixOdometryThread;

public class ModuleIOTalonSpark implements ModuleIO {
  // Hardware
  private final TalonFX driveMotor;
  private final SparkMax turnMotor;
  private final CANcoder turnCanCoder;
  private final RelativeEncoder turnRelativeEncoder;

  // Status Signals
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;
  private final StatusSignal<Temperature> driveTemp;
  private final StatusSignal<Angle> turnAbsolutePos;

  // Control Requests & Config Cache (Optimización PowerHouse)
  private final VoltageOut driveVoltageRequest = new VoltageOut(0);
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration(); // Cache para reusar

  // Constants
  private final double DRIVE_GEAR_RATIO;
  private final double TURN_GEAR_RATIO;

  public ModuleIOTalonSpark(
      int driveID,
      int turnID,
      int cancoderID,
      Rotation2d angleOffset,
      double driveGearRatio,
      double turnGearRatio,
      boolean driveInverted) {

    this.DRIVE_GEAR_RATIO = driveGearRatio;
    this.TURN_GEAR_RATIO = turnGearRatio;

    // 1. Configure Drive (TalonFX)
    driveMotor = new TalonFX(driveID);

    // Configuración inicial limpia
    // (Modificamos el objeto miembro driveConfig directamente)
    driveConfig.MotorOutput.Inverted =
        driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

    // Aplicamos la configuración inicial
    driveMotor.getConfigurator().apply(driveConfig);

    // ... (Resto del constructor para SparkMax y CANCoder igual que antes) ...
    turnMotor = new SparkMax(turnID, MotorType.kBrushless);
    turnRelativeEncoder = turnMotor.getEncoder();

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.smartCurrentLimit(30);
    turnConfig.inverted(true);
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnCanCoder = new CANcoder(cancoderID);
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = angleOffset.getRotations();
    turnCanCoder.getConfigurator().apply(cancoderConfig);

    // Signal Setup
    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getSupplyCurrent();
    driveTemp = driveMotor.getDeviceTemp();
    turnAbsolutePos = turnCanCoder.getAbsolutePosition();

    driveVelocity.setUpdateFrequency(4);
    driveAppliedVolts.setUpdateFrequency(4);
    driveCurrent.setUpdateFrequency(4);
    driveTemp.setUpdateFrequency(4);

    turnAbsolutePos.setUpdateFrequency(50);
    drivePosition.setUpdateFrequency(50);

    driveMotor.optimizeBusUtilization();

    // TODO: si mama comentar esta linea xd
    PhoenixOdometryThread.getInstance().registerSignals(drivePosition, turnAbsolutePos);
  }

  // ... (updateInputs, setDriveVoltage, setTurnVoltage igual que antes) ...
  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        drivePosition, driveVelocity, driveAppliedVolts, driveCurrent, driveTemp, turnAbsolutePos);

    inputs.drivePositionRad = drivePosition.getValue().in(Radians) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        driveVelocity.getValue().in(RadiansPerSecond) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValue().in(Volts);
    inputs.driveCurrentAmps = driveCurrent.getValue().in(Amps);
    inputs.driveTempCelcius = driveTemp.getValue().in(Celsius);

    inputs.turnPositionRad =
        Units.rotationsToRadians(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            turnRelativeEncoder.getVelocity() / TURN_GEAR_RATIO);
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
    inputs.turnCurrentAmps = turnMotor.getOutputCurrent();

    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(Units.rotationsToRadians(turnAbsolutePos.getValue().in(Rotations)));
  }

  @Override
  public void resetEncoder() {
    double absolutePositionRot =
        turnCanCoder.getAbsolutePosition().waitForUpdate(0.5).getValue().in(Rotations);
    turnRelativeEncoder.setPosition(absolutePositionRot * TURN_GEAR_RATIO);
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setControl(driveVoltageRequest.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    // Reusamos driveConfig y refrescamos para estar seguros
    driveMotor.getConfigurator().refresh(driveConfig);
    driveConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveMotor.getConfigurator().apply(driveConfig);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    turnMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setDriveOpenLoopRampRate(double timeSeconds) {
    // 1. REFRESH: Bajamos la config actual del motor al objeto cacheado
    // 'driveConfig'
    // Esto asegura que NO borremos el Current Limit ni el Inverted status.
    driveMotor.getConfigurator().refresh(driveConfig);

    // 2. MODIFY: Cambiamos solo lo que nos interesa
    driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = timeSeconds;
    driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = timeSeconds;

    // 3. APPLY: Subimos el paquete completo actualizado
    driveMotor.getConfigurator().apply(driveConfig);
  }
}
