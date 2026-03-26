package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerIntakeCommand extends Command {
  private final Indexer indexer;

  public IndexerIntakeCommand(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void execute() {
    indexer.intake();
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();
  }
}
