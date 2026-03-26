package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.DriveToPose;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Set;

/**
 * Comando de prueba para validar el giro relativo de 180 grados utilizando el control de lazo
 * cerrado (DriveToPose en Precision Mode).
 */
public class TestTurn180 extends SequentialCommandGroup {

  public TestTurn180(SwerveDrive swerve) {
    addCommands(
        // Utilizamos DeferredCommand para asegurar que la pose actual se evalúe
        // EXACTAMENTE en el momento en que este comando comienza a ejecutarse.
        new DeferredCommand(
            () -> {
              // 1. Leemos dónde estamos parados en este preciso milisegundo
              Pose2d currentPose = swerve.getPose();

              // 2. Calculamos la pose objetivo:
              // Misma posición (X, Y), pero sumando 180 grados al ángulo actual
              Pose2d targetPose =
                  new Pose2d(
                      currentPose.getTranslation(),
                      currentPose.getRotation().plus(Rotation2d.fromDegrees(180)));

              // 3. Retornamos tu DriveToPose en PrecisionMode (true)
              // pasándole un supplier que siempre devuelve esta nueva pose estática.
              return new DriveToPose(swerve, () -> targetPose, true);
            },
            // Requerimiento del subsistema para el DeferredCommand
            Set.of(swerve)));
  }
}
