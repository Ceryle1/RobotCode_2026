package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.intake.pivot.Pivot;

public class PivotWobbleCommand extends Command {
  private final Pivot pivot;
  private boolean goingDown = true; // start by going to WOBBLE_LOW

  private static final double WOBBLE_HIGH =
      HighAltitudeConstants.Pivot.WOBBLE_HIGH * HighAltitudeConstants.Pivot.GEAR_RATIO;
  private static final double WOBBLE_LOW =
      HighAltitudeConstants.Pivot.WOBBLE_LOW * HighAltitudeConstants.Pivot.GEAR_RATIO;
  private static final double TOLERANCE = HighAltitudeConstants.Pivot.WOBBLE_TOLERANCE;

  public PivotWobbleCommand(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    goingDown = true;
    pivot.setTarget(WOBBLE_LOW);
  }

  @Override
  public void execute() {
    if (pivot.atSetpoint(TOLERANCE)) {

      goingDown = !goingDown;
      pivot.setTarget(goingDown ? WOBBLE_LOW : WOBBLE_HIGH);
    }
  }

  @Override
  public void end(boolean interrupted) {
    pivot.expand();
  }
}
