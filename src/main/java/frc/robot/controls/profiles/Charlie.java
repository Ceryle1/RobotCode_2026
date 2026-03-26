package frc.robot.controls.profiles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.HighAltitudeConstants;
import frc.robot.HighAltitudeConstants.Shooter;
import frc.robot.RobotContainer;
import frc.robot.commands.indexer.IndexerEjectCommand;
import frc.robot.commands.indexer.IndexerIntakeCommand;
import frc.robot.commands.pivot.PivotExpandCommand;
import frc.robot.commands.pivot.PivotWobbleCommand;
import frc.robot.commands.shooter.AutoAimCommand;
import frc.robot.commands.shooter.FeederPreShootCommand;
import frc.robot.commands.shooter.FlywheelShootCommand;
import frc.robot.controls.ControlProfile;

public class Charlie implements ControlProfile {
  private final CommandXboxController controller;
  private final double DEADBAND = HighAltitudeConstants.Controls.DEADBAND;

  public Charlie() {
    this.controller = new CommandXboxController(1);
  }

  @Override
  public double getDriveForward() {
    // Eje Y izquierdo negado: joystick arriba = robot hacia adelante
    return -MathUtil.applyDeadband(controller.getLeftY(), DEADBAND);
  }

  @Override
  public double getDriveStrafe() {
    return -MathUtil.applyDeadband(controller.getLeftX(), DEADBAND);
  }

  @Override
  public double getDriveRotation() {
    return -MathUtil.applyDeadband(controller.getRightX(), DEADBAND);
  }

  @Override
  public void configureBindings(RobotContainer container) {
    controller
        .b()
        .whileTrue(
            new AutoAimCommand(
                container.getDrive(),
                container.getHood(),
                container.getFlywheel(),
                this::getDriveForward,
                this::getDriveStrafe));

    controller.a().onTrue(container.getHood().runOnce(() -> container.getHood().setAngle(0.0)));

    controller
        .y()
        .onTrue(
            container.getHood().runOnce(() -> container.getHood().setAngle(Math.toRadians(45.0))));

    controller.rightTrigger().whileTrue(new FeederPreShootCommand(container.getFeeder()));

    controller
        .rightTrigger()
        .whileTrue(
            new FlywheelShootCommand(
                container.getFlywheel(), Shooter.MAX_FLYWHEEL_SPEED_RADS_PER_SEC));

    controller.leftTrigger().whileTrue(new PivotWobbleCommand(container.getPivot()));
    controller.povDown().whileTrue(new PivotExpandCommand(container.getPivot()));
    controller.rightBumper().whileTrue(new IndexerIntakeCommand(container.getIndexer()));
    controller.leftBumper().whileTrue(new IndexerEjectCommand(container.getIndexer()));
  }
}
