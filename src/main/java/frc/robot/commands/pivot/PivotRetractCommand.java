package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.intake.pivot.Pivot;

public class PivotRetractCommand extends Command {
  private final Pivot pivot;

  public PivotRetractCommand(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    pivot.retract(); // Set target once, not every loop
  }

  @Override
  public boolean isFinished() {
    return pivot.atSetpoint(HighAltitudeConstants.Pivot.GEAR_RATIO * 0.01);
  }
}
