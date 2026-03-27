package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.HighAltitudeConstants.Shooter;
import frc.robot.HighAltitudeConstants.Swerve;
import frc.robot.HighAltitudeConstants.Swerve.ModuleConstants;
import frc.robot.commands.auto.SimpleDriveIntakeShoot;
import frc.robot.commands.auto.TestTurn180;
import frc.robot.commands.auto.auto1Cycle1;
import frc.robot.commands.auto.auto1Cycle3;
import frc.robot.commands.auto.auto2Cycle1;
import frc.robot.commands.auto.auto2Cycle3;
import frc.robot.commands.auto.autoLlanura;
import frc.robot.commands.auto.mhmAuto;
import frc.robot.commands.shooter.AutoAimCommand;
import frc.robot.commands.swerve.DefaultSwerveDriveNew;
import frc.robot.controls.profiles.Charlie;
import frc.robot.controls.profiles.Pato;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.Pivot;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOSparkMax;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.LedsIO;
import frc.robot.subsystems.leds.LedsIOCANdle;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerIOTalonFX;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.gyro.GyroIONavX;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOTalonSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOSim;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final SwerveDrive drive;
  private final Flywheel flywheel;
  private final Pivot intakePivot;
  private final Indexer indexer;
  private final Intake intake;
  private final Feeder feeder;
  private final Leds leds;
  private final Vision vision;
  private final Hood hood;
  private final AutoFactory autoFactory;

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {

    switch (HighAltitudeConstants.currentMode) {
      case REAL:
        Logger.recordOutput("RobotContainer/Mode", "REAL");

        // Real Hardware
        drive =
            new SwerveDrive(
                new GyroIONavX(),
                createRealModule(Swerve.MOD_FL, 0),
                createRealModule(Swerve.MOD_FR, 1),
                createRealModule(Swerve.MOD_BL, 2),
                createRealModule(Swerve.MOD_BR, 3));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_FRONT_NAME, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_BACK_NAME, VisionConstants.robotToCamera1));

        flywheel =
            new Flywheel(
                new FlywheelIOTalonFX(
                    HighAltitudeConstants.Shooter.SHOOTER_RIGHT_MOTOR_ID,
                    HighAltitudeConstants.Shooter.SHOOTER_LEFT_MOTOR_ID));

        intakePivot = new Pivot(new PivotIOSparkMax(HighAltitudeConstants.Pivot.PIVOT_MOTOR_ID));

        indexer = new Indexer(new RollerIOTalonFX(HighAltitudeConstants.Indexer.INDEXER_MOTOR_ID));

        intake = new Intake(new RollerIOTalonFX(HighAltitudeConstants.Intake.INTAKE_MOTOR_ID));

        feeder = new Feeder(new RollerIOTalonFX(HighAltitudeConstants.Feeder.FEEDER_MOTOR_ID));

        hood = new Hood(new HoodIOTalonFX(HighAltitudeConstants.Shooter.HOOD_MOTOR_ID));

        leds =
            new Leds(
                new LedsIOCANdle(
                    HighAltitudeConstants.Leds.LED_CANDLE_ID,
                    HighAltitudeConstants.Leds.LED_COUNT));

        break;

      case SIM:
        Logger.recordOutput("RobotContainer/Mode", "SIMULATION");

        // Simulation
        drive =
            new SwerveDrive(
                new GyroIOSim(),
                new SwerveModule(new ModuleIOSim(), 0),
                new SwerveModule(new ModuleIOSim(), 1),
                new SwerveModule(new ModuleIOSim(), 2),
                new SwerveModule(new ModuleIOSim(), 3));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOSim(drive::getPose),
                new VisionIOSim(drive::getPose));

        flywheel = new Flywheel(new FlywheelIO() {});
        intakePivot = new Pivot(new PivotIO() {});
        indexer = new Indexer(new RollerIO() {});
        intake = new Intake(new RollerIO() {});
        feeder = new Feeder(new RollerIO() {});
        leds = new Leds(new LedsIO() {});

        hood = new Hood(new HoodIOSim());
        break;

      default:
        Logger.recordOutput("RobotContainer/Mode", "REPLAY");
        // Replay: Interfaces vacías. AdvantageKit inyectará los logs de los inputs
        // mágicamente.

        drive =
            new SwerveDrive(
                new GyroIOSim(),
                new SwerveModule(new ModuleIOSim(), 0),
                new SwerveModule(new ModuleIOSim(), 1),
                new SwerveModule(new ModuleIOSim(), 2),
                new SwerveModule(new ModuleIOSim(), 3));

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        hood = new Hood(new HoodIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        intakePivot = new Pivot(new PivotIO() {});
        indexer = new Indexer(new RollerIO() {});
        intake = new Intake(new RollerIO() {});
        feeder = new Feeder(new RollerIO() {});
        leds = new Leds(new LedsIO() {});
        break;
    }

    boolean isRed =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;

    autoFactory =
        new AutoFactory(drive::getPose, drive::resetPose, drive::choreoPIDController, isRed, drive);

    autoFactory.bind("intake", Commands.runEnd(() -> intake.run(), () -> intake.stop(), intake));
    autoFactory.bind("shoot", new AutoAimCommand(drive, hood, flywheel, () -> 0.0, () -> 0.0));
    autoFactory.bind(
        "spinUp",
        Commands.run(
            () -> flywheel.runVelocity(Units.rotationsPerMinuteToRadiansPerSecond(4800)),
            flywheel));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    configureBindings();
    setupAutonomousCommands();
  }

  private SwerveModule createRealModule(ModuleConstants constants, int index) {
    ModuleIO io =
        new ModuleIOTalonSpark(
            constants.driveID(),
            constants.turnID(),
            constants.cancoderID(),
            constants.offset(),
            Swerve.DRIVE_GEAR_RATIO,
            Swerve.TURN_GEAR_RATIO,
            constants.driveInverted());
    return new SwerveModule(io, index);
  }

  private void configureBindings() {
    Pato driver = new Pato();
    Charlie coDriver = new Charlie();
    drive.setDefaultCommand(
        new DefaultSwerveDriveNew(
            drive,
            driver::getDriveForward, // vX
            driver::getDriveStrafe, // vY
            driver::getDriveRotation, // Omega
            driver::isPrecisionMode, // Gatillo izquierdo (Left Trigger)
            () -> true // Field Oriented siempre activado por defecto
            ));
    flywheel.setDefaultCommand(Commands.run(() -> flywheel.coast(), flywheel));
    driver.configureBindings(this);
    coDriver.configureBindings(this);
  }

  
      

  /** Configures all the auto routines available on the Dashboard. */

  // --- Rutina de Autonomos ---
  private void setupAutonomousCommands() {
    autoChooser.addDefaultOption("Do Nothing", Commands.none());

    autoChooser.addOption("Test Turn 180", new TestTurn180(drive));

    // mhm auto mrd
    autoChooser.addOption(
        "Auto Pruba mhm", new mhmAuto(autoFactory, feeder, intake).routine().cmd());

    autoChooser.addOption(
        "me pashe",
        new autoLlanura(autoFactory, feeder, intake, intakePivot, indexer, flywheel)
            .routine()
            .cmd());
    // -- Option Chooser Official Autos --
    // # Position 3 (Down) Autos #
    autoChooser.addOption(
      "auto1Cycle3", 
      new auto1Cycle3(autoFactory, feeder, intake, intakePivot, indexer, flywheel)
            .routine()
            .cmd()
      );

      autoChooser.addOption(
        "auto2Cycle3", 
        new auto2Cycle3(autoFactory, feeder, intake, intakePivot, indexer, flywheel)
            .routine()
            .cmd());

    // # Position 1 (Up) Autos #
      autoChooser.addOption(
      "auto1Cycle1", 
      new auto1Cycle1(autoFactory, feeder, intake, intakePivot, indexer, flywheel)
            .routine()
            .cmd()
      );

      autoChooser.addOption(
        "auto2Cycle1", 
        new auto2Cycle1(autoFactory, feeder, intake, intakePivot, indexer, flywheel)
            .routine()
            .cmd());



    // -- Last Resource T-T --
    autoChooser.addOption(
        "Leave and Shoot",
        new SimpleDriveIntakeShoot(drive, intakePivot, intake, flywheel, feeder, indexer));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public SwerveDrive getDrive() {
    return drive;
  }

  public Flywheel getFlywheel() {
    return flywheel;
  }

  public Pivot getPivot() {
    return intakePivot;
  }

  public Indexer getIndexer() {
    return indexer;
  }

  public Intake getIntake() {
    return intake;
  }

  public Feeder getFeeder() {
    return feeder;
  }

  public Leds getLeds() {
    return leds;
  }

  public Vision getVision() {
    return vision;
  }

  public Hood getHood() {
    return hood;
  }
}
