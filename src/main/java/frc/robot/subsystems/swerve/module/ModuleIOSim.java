package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.HighAltitudeConstants;

public class ModuleIOSim implements ModuleIO {

  // Simuladores Físicos
  private final FlywheelSim driveSim;
  private final DCMotorSim turnSim;

  // Estado de Voltajes
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  // Simulación de Ramp Rate (Para igualar el OpenLoopRamp del TalonFX)
  // Iniciamos con un valor muy alto (sin límite) por defecto
  private SlewRateLimiter driveRampLimiter = null;

  public ModuleIOSim() {
    System.out.println("[Init] Creating ModuleIOSim High-Fidelity");

    // --- DRIVE SIMULATION (Kraken X60) ---
    // Usamos createFlywheelSystem para modelar la masa rotacional
    var drivePlant =
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(1),
            HighAltitudeConstants.MOI_DRIVE_KG_M2,
            HighAltitudeConstants.Swerve.DRIVE_GEAR_RATIO);

    driveSim =
        new FlywheelSim(
            drivePlant, DCMotor.getKrakenX60(1), 0.0 // Ruido cero para odometría limpia
            );

    // --- TURN SIMULATION (NEO) ---
    // Usamos createDCMotorSystem para modelar posición y velocidad
    var turnPlant =
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1),
            HighAltitudeConstants.MOI_TURN_KG_M2,
            HighAltitudeConstants.Swerve.TURN_GEAR_RATIO);

    turnSim =
        new DCMotorSim(
            turnPlant, DCMotor.getNEO(1), 0.0, 0.0 // Ruido cero
            );
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // 1. Simular la caída de voltaje de la batería (Battery Sag)
    // Obtenemos el voltaje simulado actual (que baja si otros motores consumen
    // corriente)
    double batteryVoltage = RobotController.getBatteryVoltage();

    // Clampeamos el voltaje solicitado para no exceder lo que la batería puede dar
    // realmente
    double driveVoltsLimited = MathUtil.clamp(driveAppliedVolts, -batteryVoltage, batteryVoltage);
    double turnVoltsLimited = MathUtil.clamp(turnAppliedVolts, -batteryVoltage, batteryVoltage);

    // 2. Aplicar Ramp Rate (Si está configurado)
    // Esto simula el "OpenLoopRamp" del TalonFX
    if (driveRampLimiter != null) {
      driveVoltsLimited = driveRampLimiter.calculate(driveVoltsLimited);
    }

    // 3. Aplicar Voltajes a la Física
    driveSim.setInputVoltage(driveVoltsLimited);
    turnSim.setInputVoltage(turnVoltsLimited);

    // 4. Avanzar Física (20ms)
    driveSim.update(0.02);
    turnSim.update(0.02);

    // 5. Output a Inputs (Sensores Simulados)

    // Drive (FlywheelSim es velocidad -> Integramos posición manualmente)
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.drivePositionRad += inputs.driveVelocityRadPerSec * 0.02;

    // Reportamos el voltaje REAL que llegó al motor (después de batería y ramp)
    inputs.driveAppliedVolts = driveVoltsLimited;
    inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();

    // Turn (DCMotorSim ya maneja posición)
    inputs.turnPositionRad = turnSim.getAngularPositionRad();
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnVoltsLimited;
    inputs.turnCurrentAmps = turnSim.getCurrentDrawAmps();

    // Encoder Absoluto (Perfecto en Sim)
    inputs.turnAbsolutePositionRad = inputs.turnPositionRad;
  }

  @Override
  public void setDriveVoltage(double volts) {
    // Solo guardamos la intención. El updateInputs aplica la lógica de
    // batería/ramp.
    this.driveAppliedVolts = volts;
  }

  @Override
  public void setTurnVoltage(double volts) {
    this.turnAppliedVolts = volts;
  }

  @Override
  public void setDriveOpenLoopRampRate(double timeSeconds) {
    // Si timeSeconds es 0, desactivamos el limiter
    if (timeSeconds <= 0) {
      driveRampLimiter = null;
    } else {
      // SlewRateLimiter toma "unidades por segundo".
      // Si queremos ir de 0V a 12V en 'timeSeconds', la tasa es 12 / timeSeconds.
      // Asumimos 12V nominales para el cálculo de la tasa.
      double rate = 12.0 / timeSeconds;
      driveRampLimiter = new SlewRateLimiter(rate);
    }
  }

  // Opcional: Simulación básica de Brake/Coast (FlywheelSim no tiene frenado
  // activo real,
  // pero podríamos simularlo cambiando ganancias. Por ahora lo dejamos vacío ya
  // que
  // la inercia (J) domina el comportamiento).
  @Override
  public void setDriveBrakeMode(boolean enable) {}

  @Override
  public void setTurnBrakeMode(boolean enable) {}
}
