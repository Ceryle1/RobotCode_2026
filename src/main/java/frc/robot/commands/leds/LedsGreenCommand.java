package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.Leds;

public class LedsGreenCommand extends Command {
  private final Leds leds;

  public LedsGreenCommand(Leds leds) {
    this.leds = leds;
  }

  @Override
  public void initialize() {
    leds.green();
  }

  @Override
  public void end(boolean interrupted) {
    leds.off();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
