package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HighAltitudeConstants;
import frc.robot.commands.indexer.IndexerIntakeCommand;
import frc.robot.commands.intake.IntakeAbsorbCommand;
import frc.robot.commands.pivot.PivotExpandCommand;
import frc.robot.commands.shooter.FeederPreShootCommand;
import frc.robot.commands.shooter.FlywheelShootCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.Pivot;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Auto simple: 1. Moverse hacia atrás directamente con los módulos. 2. Expandir intake y succionar.
 * 3. Encender Flywheel y disparar. 4. Girar 180 grados sobre su propio eje.
 */
public class SimpleDriveIntakeShootTurn extends SequentialCommandGroup {

  public SimpleDriveIntakeShootTurn(
      SwerveDrive swerve,
      Pivot pivot,
      Intake intake,
      Flywheel flywheel,
      Feeder feeder,
      Indexer indexer) {

    addCommands(
        // 1. Moverse hacia atrás a -0.75 m/s por 2.5 segundos, luego detenerse.
        new RunCommand(() -> swerve.driveBackwardsDirect(-0.75), swerve)
            .withTimeout(2.5)
            .andThen(new InstantCommand(() -> swerve.stop(), swerve)),

        // 2. Bajar intake y prender rodillos
        new SequentialCommandGroup(new PivotExpandCommand(pivot), new IntakeAbsorbCommand(intake))
            .withTimeout(1.5),

        // 3. Preparar disparo: Encender Flywheel y disparar con delay
        new ParallelCommandGroup(
                new FlywheelShootCommand(
                    flywheel, HighAltitudeConstants.Shooter.MAX_FLYWHEEL_SPEED_RADS_PER_SEC),
                new SequentialCommandGroup(
                    new WaitCommand(3.0),
                    new ParallelCommandGroup(
                        new FeederPreShootCommand(feeder), new IndexerIntakeCommand(indexer))))
            .withTimeout(
                4.5), // Le damos un timeout global al grupo de disparo para que pueda avanzar al
        // giro

        // Asegurar que todo se apague antes de girar (opcional pero recomendado)
        new InstantCommand(
            () -> {
              flywheel.coast();
              feeder.stop();
              indexer.stop();
            }),

        // 4. GIRAR 180 GRADOS (PI radianes)
        // Giramos a PI rad/s durante 1 segundo = PI radianes (180 grados) de desplazamiento
        new RunCommand(() -> swerve.runVelocity(new ChassisSpeeds(0.0, 0.0, Math.PI)), swerve)
            .withTimeout(1.0)
            .andThen(new InstantCommand(() -> swerve.stop(), swerve)));
  }
}
