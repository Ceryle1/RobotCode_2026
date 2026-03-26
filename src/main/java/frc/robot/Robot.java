// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  public Robot() {
    // Record metadata
    /*
     * TODO: Coding missing
     *
     * *Swerve:
     * - DriveToPose (TOPP), have two sets of gain values, one for speed and other
     * for precission
     * - Swerve SysID (Drive) (Turn?)
     * - Swerve SnapToHub & SnapToFeed (turnTo)
     * - SetUp for a future ShootOnTheFly (methods and stuff missing)
     *
     * RobotStates: Intaking, ShootingHub, ShootingFeed, Climbing(Dividing Auto or
     * TeleOp), SwerveMotion(For ShootOnTheFly), no sé cuáles faltarían
     *
     * *Vision:
     * - Implement things from RobotCode_2025 and PowerHouse stuff
     * - Main Objective: SwerveOdometryPose Feder
     *
     * Subsystems:
     * -Requirements for every single one:
     * -Hardware Abstraction Layer (KrakenX60, KrakenX44, NEO SparkMax,s NEOVortex
     * SparkFlex) useful when a part is swapped
     * -HighAltitudeConstants with ID Values, Inverted Values, Speeds, Gain Values,
     * Setpoints, TypeOfMotor, etc. (changes adapted to each subsystem)
     *
     * *Shooter:
     * - Creating the subsystem and methods (basic & advanced ones)
     * - SysID for PID (RPM) and FF for faster accel
     * - Logging setpoints and actualSpeed for FineTuning
     *
     * *Indexer:
     * - Creating the subsystem and methods (basic & advanced ones)
     * - SysID for FF, tune two (if needed) FF, one for intaking and the other one
     * for feeding the Shooter
     * - Log only the neccessarie
     *
     * *Intake:
     * - Creating the subsystem and methods (basic & advanced ones)
     * - SysID for FF
     *
     * *IntakePivot:
     * - Creating the subsystem and methods (basic & advanced ones)
     * - SysID for PID
     * - Create the Mechanism3D for Simulation/Replay?
     *
     * *Climber:
     * - Creating the subsystem and methods (basic & advanced ones)
     * - FF for the climber in order to not fall down
     * - Reminder: Climb in auto, then go down unclimb, and play all teleop, at
     * endgame reclimb
     * - Create the Mechanism3D for Simulation/Replay?
     *
     * *POSSIBLE: ShooterHood/Pivot:
     * - Creating the subsystem and methods (basic & advanced ones)
     * - See where to create the AutoAim, based on the RobotPose & RobotMode return
     * a HoodAngle, for better results we are keeping the same RPM for both methods
     * and all lengths
     * - Create the Mechanism3D for Simulation/Replay?
     *
     */
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (HighAltitudeConstants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println("The auto choosed is " + m_autonomousCommand.getName());
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
