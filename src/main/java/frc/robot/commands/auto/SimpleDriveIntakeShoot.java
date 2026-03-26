package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.HighAltitudeConstants;
import frc.robot.commands.indexer.IndexerIntakeCommand;
import frc.robot.commands.shooter.FeederPreShootCommand;
import frc.robot.commands.shooter.FlywheelShootCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.Pivot;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Auto simple (Direct Module Control): 1. Moverse hacia atrás directamente con los módulos (sin PID
 * de odometría). 2. Expandir intake y succionar. 3. Encender Flywheel y disparar mediante el
 * Feeder.
 */
public class SimpleDriveIntakeShoot extends SequentialCommandGroup {

  public SimpleDriveIntakeShoot(
      SwerveDrive swerve,
      Pivot pivot,
      Intake intake,
      Flywheel flywheel,
      Feeder feeder,
      Indexer indexer) {

    addCommands(
        // 1. Moverse hacia atrás a 1.5 m/s por 1.0 segundo, luego detenerse.
        new RunCommand(() -> swerve.driveBackwardsDirect(-0.75), swerve)
            .withTimeout(2.5)
            .andThen(new InstantCommand(() -> swerve.stop(), swerve)),

        // 2. Bajar intake y prender rodillos en paralelo
        /*  new SequentialCommandGroup(new PivotExpandCommand(pivot), new IntakeAbsorbCommand(intake))
            .withTimeout(1.5), // Tiempo para asegurar que bajó y tomó la
        // pieza*/

        // 3. Preparar disparo: Encender Flywheel
        new ParallelCommandGroup(
            new FlywheelShootCommand(
                flywheel, HighAltitudeConstants.Shooter.MAX_FLYWHEEL_SPEED_RADS_PER_SEC),

            // 4. Esperar a que el Flywheel esté en velocidad (tolerancia de 10 rad/s)

            // 5. Disparar activando el Feeder
            new SequentialCommandGroup(
                // 4. Esperar a que el Flywheel esté en velocidad (tolerancia de 10 rad/s)
                new WaitCommand(3),
                new ParallelCommandGroup(
                    new FeederPreShootCommand(feeder), new IndexerIntakeCommand(indexer))),

            // 6. Timeout de seguridad
            new WaitCommand(4.0)),
        new InstantCommand(
            () -> {
              // A) Redefinir el frente
              // swerve.zeroGyro();

              // B) Detener absolutamente todos los motores de forma segura
              swerve.stop();
              intake.stop();
              flywheel.coast(); // Flywheel usa coast() en lugar de stop() para no frenar de golpe
              feeder.stop();
              indexer.stop();
            }));
  }
}
