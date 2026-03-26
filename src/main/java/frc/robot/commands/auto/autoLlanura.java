package frc.robot.commands.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.Pivot;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;

public class autoLlanura {
  private final AutoRoutine routine;

  public autoLlanura(
      AutoFactory autoFactory,
      Feeder feeder,
      Intake intake,
      Pivot pivot,
      Indexer indexer,
      Flywheel flywheel) {
    routine = autoFactory.newRoutine("autoLlanura");

    AutoTrajectory traj = routine.trajectory("autoLlanura");

    routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));

    traj.atTime("shoot1")
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(() -> flywheel.runVelocity(300), flywheel),
                    new WaitCommand(1.5),
                    Commands.parallel(
                            Commands.runEnd(() -> feeder.intake(), () -> feeder.stop(), feeder),
                            Commands.runEnd(() -> indexer.intake(), () -> indexer.stop(), indexer))
                        .withTimeout(2.0),
                    Commands.runOnce(
                        () -> {
                          flywheel.coast();
                          feeder.stop();
                          indexer.stop();
                        },
                        flywheel,
                        feeder,
                        indexer))
                .withTimeout(5));

    traj.atTime("intake1")
        .onTrue(
            Commands.sequence(
                Commands.parallel(
                        Commands.runOnce(() -> intake.run(), intake),
                        Commands.runOnce(() -> pivot.expand(), pivot))
                    .withTimeout(2.0)));
  }

  public AutoRoutine routine() {
    return routine;
  }
}
