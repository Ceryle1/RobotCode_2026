package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeEjectCommand extends Command {
  private final Intake intake;

  public IntakeEjectCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.eject();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
