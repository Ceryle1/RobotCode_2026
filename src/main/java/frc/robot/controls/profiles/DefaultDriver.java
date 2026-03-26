package frc.robot.controls.profiles;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.Shooter;
import frc.robot.RobotContainer;
import frc.robot.commands.indexer.IndexerIntakeCommand;
import frc.robot.commands.intake.IntakeAbsorbCommand;
import frc.robot.commands.pivot.PivotExpandCommand;
import frc.robot.commands.pivot.PivotWobbleCommand;
import frc.robot.commands.shooter.FeederPreShootCommand;
import frc.robot.commands.shooter.FlywheelShootCommand;
import frc.robot.commands.swerve.DriveToPose;
import frc.robot.commands.swerve.SwerveHeadingLockCommand;
import frc.robot.controls.ControlProfile;
import java.util.Set;
import java.util.function.Supplier;

public class DefaultDriver implements ControlProfile {
  private final CommandXboxController controller;
  private final double DEADBAND = HighAltitudeConstants.Controls.DEADBAND;

  public DefaultDriver() {
    // Puerto 0 por defecto para el Driver
    this.controller = new CommandXboxController(0);
  }

  @Override
  public double getDriveForward() {
    // Joystick Y negativo es "arriba". Invertimos para que + sea adelante.
    return -controller.getLeftY();
  }

  @Override
  public double getDriveStrafe() {
    // Joystick X positivo es "derecha". Invertimos para que + sea izquierda (NWU).
    return -controller.getLeftX();
  }

  @Override
  public double getDriveRotation() {
    // Joystick X positivo es "derecha". Invertimos para que + sea CCW.
    return -controller.getRightX();
  }

  public boolean isPrecisionMode() {
    return controller.povRight().getAsBoolean();
  }

  @Override
  public void configureBindings(RobotContainer container) {

    controller.leftTrigger().whileTrue(new IntakeAbsorbCommand(container.getIntake()));
    controller.leftBumper().whileTrue(new IndexerIntakeCommand(container.getIndexer()));
    controller.leftBumper().whileTrue(new FeederPreShootCommand(container.getFeeder()));
    controller
        .rightTrigger()
        .whileTrue(
            new FlywheelShootCommand(
                container.getFlywheel(), Shooter.MAX_FLYWHEEL_SPEED_RADS_PER_SEC));

    controller.povDown().onTrue(new PivotExpandCommand(container.getPivot()));
    controller.povUp().whileTrue(new PivotWobbleCommand(container.getPivot()));
    // Before

    // --- HOOD MANUAL TESTING ---
    // Gatillo Izquierdo: Manda el Hood a su posición máxima (45 grados)
    controller
        .leftTrigger()
        .onTrue(
            container.getHood().runOnce(() -> container.getHood().setAngle(Math.toRadians(45.0))));

    // Botón Back (Select): Regresa el Hood a 0 grados
    controller.back().onTrue(container.getHood().runOnce(() -> container.getHood().setAngle(0.0)));

    // --- DRIVER ASSISTS ---

    // Botón X: DriveToPose (PID Local) - Ajuste Fino
    controller
        .x()
        .whileTrue(
            new DriveToPose(
                container.getDrive(),
                () -> FieldConstants.onAlliance(FieldConstants.BLUE_SHOOTING_LEFT),
                false));

    // Botón B: PathFinding (PathPlanner A*) - Navegación Global
    // Ideal para cruzar la cancha evadiendo obstáculos.
    // Usamos un Supplier () -> ... para que calcule la alianza al presionar el
    // botón.
    controller
        .b()
        .whileTrue(
            getPathFindCommand(
                container, () -> FieldConstants.onAlliance(FieldConstants.BLUE_SHOOTING_LEFT)));

    // Botón Y: Snap to HUB (Centro) - Orbital Mode
    // Permite orbitar el Hub disparando.
    controller
        .y()
        .whileTrue(
            new SwerveHeadingLockCommand(
                container.getDrive(),
                () -> -controller.getLeftY(), // Forward
                () -> -controller.getLeftX(), // Strafe
                () -> FieldConstants.onAlliance(FieldConstants.BLUE_HUB_CENTER) // Target Dinámico
                ));

    // Botón A: Snap to FEEDER / ALLIANCE WALL (180 Grados)
    // Apunta hacia atrás para recoger game pieces o defender zona.
    controller
        .a()
        .whileTrue(
            new SwerveHeadingLockCommand(
                container.getDrive(),
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> FieldConstants.onAlliance(FieldConstants.BLUE_ALLIANCE_WALL_TARGET)));

    // Reset Gyro (Start Button)
    controller.start().onTrue(container.getDrive().runOnce(() -> container.getDrive().zeroGyro()));
  }

  /**
   * Genera un comando de Pathfinding "On-the-Fly" usando PathPlanner. Usa Commands.defer para
   * asegurar que el objetivo se calcule en el momento exacto que se presiona el botón (vital para
   * Alliance Flip).
   */
  private Command getPathFindCommand(
      RobotContainer container, Supplier<Pose2d> targetPoseSupplier) {
    // Commands.defer retrasa la creación del comando hasta que se schedulea.
    return Commands.defer(
        () -> {
          // 1. Obtener el Target actualizado (Checa Alianza aquí)
          Pose2d target = targetPoseSupplier.get();

          // 2. Definir Constraints de Teleop (Más seguros que Auto)
          PathConstraints constraints =
              new PathConstraints(
                  3.0, // Max Vel (m/s) - Rápido pero controlable
                  2.5, // Max Accel (m/s^2) - Suave para no voltearse
                  Units.degreesToRadians(360), // Max Ang Vel
                  Units.degreesToRadians(540) // Max Ang Accel
                  );

          // 3. Generar la ruta A*
          // AutoBuilder ya fue configurado en SwerveDrive.java
          return AutoBuilder.pathfindToPose(
              target, constraints, 0.0 // Velocidad final 0 (llegar y parar)
              );
        },
        Set.of(container.getDrive())); // Declaramos que este comando usará el Drive
  }

  /** Aplica Deadband y Squaring para mayor precisión. */
  private double processAxis(double value) {
    // 1. Aplicar Deadband
    value = MathUtil.applyDeadband(value, DEADBAND);

    // 2. Elevar al cuadrado manteniendo el signo (Squaring)
    return Math.copySign(value * value, value);
  }
}
