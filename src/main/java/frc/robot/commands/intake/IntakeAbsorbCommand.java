package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeAbsorbCommand extends Command {
  private final Intake intake;

  public IntakeAbsorbCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.run();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
