package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Flywheel;

public class FlywheelShootCommand extends Command {
  private final Flywheel flywheel;
  private final double rpm;

  public FlywheelShootCommand(Flywheel flywheel, double rpm) {
    this.flywheel = flywheel;
    this.rpm = rpm;
    addRequirements(flywheel);
  }

  @Override
  public void execute() {
    flywheel.runVelocity(rpm);
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.coast();
  }
}
