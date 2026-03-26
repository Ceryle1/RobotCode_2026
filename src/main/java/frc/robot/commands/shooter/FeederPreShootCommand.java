package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Feeder;

public class FeederPreShootCommand extends Command {
  private final Feeder feeder;

  public FeederPreShootCommand(Feeder feeder) {
    this.feeder = feeder;
    addRequirements(feeder);
  }

  @Override
  public void execute() {
    feeder.intake();
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
  }
}
