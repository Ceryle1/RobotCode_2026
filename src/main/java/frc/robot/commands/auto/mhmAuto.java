package frc.robot.commands.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Feeder;

public class mhmAuto {

  private final AutoRoutine routine;

  public mhmAuto(AutoFactory autoFactory, Feeder feeder, Intake intake) {
    routine = autoFactory.newRoutine("autoYa");

    AutoTrajectory traj = routine.trajectory("autoYa");

    routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));
  }

  public AutoRoutine routine() {
    return routine;
  }
}
