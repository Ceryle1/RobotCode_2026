package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  // Controllers
  private final ProfiledPIDController turnPID;
  private final PIDController drivePID;
  private final SimpleMotorFeedforward driveFF;

  public SwerveModule(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    turnPID =
        new ProfiledPIDController(
            HighAltitudeConstants.Swerve.TURN_KP,
            HighAltitudeConstants.Swerve.TURN_KI,
            HighAltitudeConstants.Swerve.TURN_KD,
            new TrapezoidProfile.Constraints(
                HighAltitudeConstants.Swerve.TURN_MAX_VELOCITY_RAD_S,
                HighAltitudeConstants.Swerve.TURN_MAX_ACCELERATION_RAD_S2));

    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    drivePID =
        new PIDController(
            HighAltitudeConstants.Swerve.DRIVE_KP,
            HighAltitudeConstants.Swerve.DRIVE_KI,
            HighAltitudeConstants.Swerve.DRIVE_KD);

    driveFF =
        new SimpleMotorFeedforward(
            HighAltitudeConstants.Swerve.DRIVE_KS, HighAltitudeConstants.Swerve.DRIVE_KV);

    resetEncoder();

    io.updateInputs(inputs);

    turnPID.reset(inputs.turnPositionRad);

    // CONFIGURACIÓN DE SEGURIDAD (Opción A)
    // Configuramos la rampa en el hardware al iniciar.
    // 0.25s es un estándar seguro para evitar dañar engranajes MK4i.
    io.setDriveOpenLoopRampRate(0.25);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public void runSetpoint(SwerveModuleState state) {
    // 1. OPTIMIZACIÓN (WPILib 2026 Standard)
    Rotation2d currentAngle = new Rotation2d(inputs.turnPositionRad);

    // CORRECCIÓN: El método optimize ahora modifica 'state' directamente y regresa
    // void.
    state.optimize(currentAngle);

    // Por claridad, podemos seguir usando la variable 'optimizedState' apuntando al
    // mismo objeto
    SwerveModuleState optimizedState = state;

    // 2. COSINE COMPENSATION (Anti-Skid)
    Rotation2d error = optimizedState.angle.minus(currentAngle);
    double cosineScale = error.getCos();

    double targetSpeed = optimizedState.speedMetersPerSecond * Math.max(0.0, cosineScale);

    // 3. CONTROL LAZO CERRADO
    double turnVolts = turnPID.calculate(inputs.turnPositionRad, optimizedState.angle.getRadians());

    double currentVelocity =
        inputs.driveVelocityRadPerSec * HighAltitudeConstants.Swerve.WHEEL_RADIUS_METERS;
    double driveVolts =
        driveFF.calculate(targetSpeed) + drivePID.calculate(currentVelocity, targetSpeed);

    turnVolts =
        MathUtil.clamp(
            turnVolts, -HighAltitudeConstants.MAX_VOLTAGE, HighAltitudeConstants.MAX_VOLTAGE);
    driveVolts =
        MathUtil.clamp(
            driveVolts, -HighAltitudeConstants.MAX_VOLTAGE, HighAltitudeConstants.MAX_VOLTAGE);

    io.setTurnVoltage(turnVolts);
    io.setDriveVoltage(driveVolts);

    Logger.recordOutput(
        "Drive/Module" + index + "/SetpointVelocity", optimizedState.speedMetersPerSecond);
    Logger.recordOutput(
        "Drive/Module" + index + "/SetpointAngle", optimizedState.angle.getRadians());
  }

  // --- GETTERS & SETTERS ---

  public SwerveModulePosition getPosition() {
    double distanceMeters =
        inputs.drivePositionRad * HighAltitudeConstants.Swerve.WHEEL_RADIUS_METERS;
    return new SwerveModulePosition(distanceMeters, new Rotation2d(inputs.turnPositionRad));
  }

  public void updatePosition(SwerveModulePosition positionToUpdate) {
    positionToUpdate.distanceMeters =
        inputs.drivePositionRad * HighAltitudeConstants.Swerve.WHEEL_RADIUS_METERS;
    positionToUpdate.angle = Rotation2d.fromRadians(inputs.turnAbsolutePositionRad);
  }

  public SwerveModuleState getState() {
    double velocityMeters =
        inputs.driveVelocityRadPerSec * HighAltitudeConstants.Swerve.WHEEL_RADIUS_METERS;
    return new SwerveModuleState(velocityMeters, new Rotation2d(inputs.turnPositionRad));
  }

  public void stop() {
    io.setDriveVoltage(0.0);
    io.setTurnVoltage(0.0);
  }
}
