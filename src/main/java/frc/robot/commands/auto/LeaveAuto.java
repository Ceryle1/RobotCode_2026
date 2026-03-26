// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveAuto extends SequentialCommandGroup {
  SwerveDrive swerve;

  /** Creates a new LeaveAuto. */
  public LeaveAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand()F, new BarCommand());
    addCommands(
        new RunCommand(() -> swerve.driveBackwardsDirect(1.5), swerve)
            .withTimeout(1.0)
            .andThen(new InstantCommand(() -> swerve.stop(), swerve)));
  }
}
