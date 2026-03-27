package frc.robot.commands.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.Pivot;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;

public class auto1Cycle1 {
  private final AutoRoutine routine;

  public auto1Cycle1(
      AutoFactory autoFactory,
      Feeder feeder,
      Intake intake,
      Pivot pivot,
      Indexer indexer,
      Flywheel flywheel) {
    routine = autoFactory.newRoutine("auto1Cycle1");

    AutoTrajectory traj = routine.trajectory("auto1Cycle1");

    routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));

    traj.atTime("shoot1")
        .onTrue(
             Commands.sequence(
                    Commands.runOnce(() -> flywheel.runVelocity(Units.rotationsPerMinuteToRadiansPerSecond(4800)), flywheel),
                    new WaitUntilCommand(() -> flywheel.onTarget(4800)),
                    Commands.parallel(
                            Commands.runEnd(() -> feeder.intake(), () -> feeder.stop(), feeder),
                            Commands.runEnd(() -> indexer.intake(), () -> indexer.stop(), indexer),
                            new WaitCommand(4.5))
                        .withTimeout(4.0),
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

    traj.atTime("intakeOff1")
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> intake.stop(), intake)
            )
        );

    traj.atTime("shoot2")
        .onTrue(
             Commands.sequence(
                    Commands.runOnce(() -> flywheel.runVelocity(Units.rotationsPerMinuteToRadiansPerSecond(4800)), flywheel),
                    new WaitUntilCommand(() -> flywheel.onTarget(4800)),
                    Commands.parallel(
                            Commands.runEnd(() -> feeder.intake(), () -> feeder.stop(), feeder),
                            Commands.runEnd(() -> indexer.intake(), () -> indexer.stop(), indexer),
                            new WaitCommand(4.5))
                        .withTimeout(4.0),
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
  }

  

  public AutoRoutine routine() {
    return routine;
  }
}
