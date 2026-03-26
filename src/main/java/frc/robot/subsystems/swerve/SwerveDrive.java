package frc.robot.subsystems.swerve;

/*
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
 */
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModule[] modules;
  private final SwerveDrivePoseEstimator poseEstimator;

  // Cache de posiciones para no crear arrays en cada loop (Optimización de
  // Memoria)
  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

  // Chorus 2 PID'S Nota: Utiliza 2 por que Chorus utiliza una "sample" de x/y así que para evitar
  // cruzes de datos entre ambos valores, se usan 2 PID's
  private final PIDController PIDXController =
      new PIDController(
          HighAltitudeConstants.Auto.PATHPLANNER_TRANSLATION_KP,
          HighAltitudeConstants.Auto.PATHPLANNER_TRANSLATION_KI,
          HighAltitudeConstants.Auto.PATHPLANNER_TRANSLATION_KD);

  private final PIDController PIDYController =
      new PIDController(
          HighAltitudeConstants.Auto.PATHPLANNER_TRANSLATION_KP,
          HighAltitudeConstants.Auto.PATHPLANNER_TRANSLATION_KI,
          HighAltitudeConstants.Auto.PATHPLANNER_TRANSLATION_KD);

  private final PIDController PIDTurnController =
      new PIDController(
          HighAltitudeConstants.Auto.PATHPLANNER_TRANSLATION_KP,
          HighAltitudeConstants.Auto.PATHPLANNER_TRANSLATION_KI,
          HighAltitudeConstants.Auto.PATHPLANNER_TRANSLATION_KD);

  // Choreo AutoFactory UnU
  private final AutoFactory autoFactory;

  public SwerveDrive(GyroIO gyroIO, SwerveModule... modules) {
    this.gyroIO = gyroIO;
    this.modules = modules;

    // Inicializar array de cache
    for (int i = 0; i < 4; i++) {
      modulePositions[i] = new SwerveModulePosition();
    }

    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            HighAltitudeConstants.Swerve.KINEMATICS,
            new Rotation2d(),
            getModulePositions(), // Usa el método optimizado
            new Pose2d());

    PIDTurnController.enableContinuousInput(-Math.PI, Math.PI);
    this.autoFactory = configureChoreo();
  }
  /*
    configureAutoBuilder();
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          Logger.recordOutput("PathPlanner/ActivePath", poses.toArray(new Pose2d[0]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          Logger.recordOutput("PathPlanner/TargetPose", pose);
        });
  }
        */

  @Override
  public void periodic() {
    if (HighAltitudeConstants.currentMode == HighAltitudeConstants.Mode.SIM
        && gyroIO instanceof GyroIOSim) {
      // Calcula qué tan rápido están haciendo girar el robot las llantas
      ChassisSpeeds chassisSpeeds =
          HighAltitudeConstants.Swerve.KINEMATICS.toChassisSpeeds(getModuleStates());
      ((GyroIOSim) gyroIO).setYawData(chassisSpeeds.omegaRadiansPerSecond);
    }

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (SwerveModule module : modules) {
      module.periodic();
    }

    // --- ODOMETRÍA POWERHOUSE ---
    poseEstimator.update(Rotation2d.fromRadians(gyroInputs.yawPositionRad), getModulePositions());

    Logger.recordOutput("Odometry/Robot", getPose());
    Logger.recordOutput("Odometry/ModuleStates", getModuleStates());
  }

  // --- Configuración del Choreo ---

  boolean isRed =
      DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

  private AutoFactory configureChoreo() {
    return new AutoFactory(this::getPose, this::resetPose, this::choreoPIDController, isRed, this);
  }

  public void choreoPIDController(SwerveSample sample) {
    Pose2d currentPose = getPose();

    // Corrección PID sobre el error de posición/rotación
    double correctionX = PIDXController.calculate(currentPose.getX(), sample.x);
    double correctionY = PIDYController.calculate(currentPose.getY(), sample.y);
    double correctionTheta =
        PIDTurnController.calculate(currentPose.getRotation().getRadians(), sample.heading);

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            sample.vx + correctionX,
            sample.vy + correctionY,
            sample.omega + correctionTheta,
            currentPose.getRotation());

    runVelocity(speeds);
  }
  // PARA QUE SRIVA EN ROBOT CONTAINER
  public AutoFactory getAutoFactory() {
    return autoFactory;
  }

  /** Control principal de velocidad. */
  public void runVelocity(ChassisSpeeds speeds) {
    // CORRECCIÓN POWERHOUSE: Discretize the setpoint
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    SwerveModuleState[] setpointStates =
        HighAltitudeConstants.Swerve.KINEMATICS.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  /**
   * [NEW] Control de velocidad con centro de rotación variable. Útil para rotar alrededor de una
   * esquina del robot o un game piece.
   */
  public void runVelocity(ChassisSpeeds speeds, Translation2d centerOfRotation) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    SwerveModuleState[] setpointStates =
        HighAltitudeConstants.Swerve.KINEMATICS.toSwerveModuleStates(
            discreteSpeeds, centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  /**
   * [NUEVO] Mueve el robot hacia atrás saltándose la cinemática (comunicación directa).
   *
   * @param speedMetersPerSecond Velocidad en m/s (debe ser positiva, el ángulo da la dirección)
   */
  public void driveBackwardsDirect(double speedMetersPerSecond) {
    // Apuntamos todas las ruedas a 180 grados (hacia atrás)
    SwerveModuleState backwardState =
        new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(180));

    SwerveModuleState[] states =
        new SwerveModuleState[] {
          backwardState, // FL
          backwardState, // FR
          backwardState, // BL
          backwardState // BR
        };

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(states[i]);
    }
  }

  /**
   * [NEW] Retorna la velocidad del robot RELATIVA AL CAMPO. Requerido para DriveToPose y
   * PathPlanner.
   */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    // 1. Obtener velocidades relativas al robot
    ChassisSpeeds robotSpeeds =
        HighAltitudeConstants.Swerve.KINEMATICS.toChassisSpeeds(getModuleStates());
    // 2. Rotar por el ángulo actual para obtener velocidades de campo
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, getRotation());
  }

  /**
   * Retorna la velocidad actual relativa al robot. Requerido por PathPlanner para corregir errores.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    // kinematics.toChassisSpeeds toma ModuleStates y retorna RobotRelativeSpeeds
    return HighAltitudeConstants.Swerve.KINEMATICS.toChassisSpeeds(getModuleStates());
  }
  /*
  private void configureAutoBuilder() {
    try {
      // 1. Definir Configuración Física del Robot (Para Choreo/PathPlanner)
      // Usa la masa, inercia y fricción del módulo para calcular Feedforwards
      // precisos
      RobotConfig config =
          new RobotConfig(
              HighAltitudeConstants.MASS_KG,
              HighAltitudeConstants.MOI_KG_M2,
              new com.pathplanner.lib.config.ModuleConfig(
                  HighAltitudeConstants.WHEEL_RADIUS_METERS,
                  HighAltitudeConstants.Swerve.MAX_LINEAR_SPEED_M_S,
                  1.2, // Coeficiente de fricción de rueda (1.2 es estándar para carpet)
                  new DCMotor(
                      12, 7.09, 366, 2, Units.rotationsPerMinuteToRadiansPerSecond(6000), 1),
                  HighAltitudeConstants.Swerve.DRIVE_GEAR_RATIO, // Drive Gearing
                  40.0, // Current Limit
                  1 // FNum motors per module
                  ),
              HighAltitudeConstants.Swerve.KINEMATICS.getModules() // Module offsets
              );

      // 2. Configurar AutoBuilder
      AutoBuilder.configure(
          this::getPose, // Supplier<Pose2d>
          this::resetPose, // Consumer<Pose2d>
          this::getRobotRelativeSpeeds, // Supplier<ChassisSpeeds> (Nuevo método abajo)

          // Output Consumer: Usamos runVelocity para respetar tu HAL
          (speeds, feedforwards) -> runVelocity(speeds),
          new PPHolonomicDriveController(
              // PID Traslación
              new PIDConstants(
                  HighAltitudeConstants.Auto.PATHPLANNER_TRANSLATION_KP,
                  HighAltitudeConstants.Auto.PATHPLANNER_TRANSLATION_KI,
                  HighAltitudeConstants.Auto.PATHPLANNER_TRANSLATION_KD),
              // PID Rotación
              new PIDConstants(
                  HighAltitudeConstants.Auto.PATHPLANNER_ROTATION_KP,
                  HighAltitudeConstants.Auto.PATHPLANNER_ROTATION_KI,
                  HighAltitudeConstants.Auto.PATHPLANNER_ROTATION_KD)),
          config, // Configuración física
          () -> {
            // Boolean supplier for alliance mirroring
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this // Subsystem requirement
          );

    } catch (Exception e) {
      System.err.println("CRITICAL: Failed to configure AutoBuilder. PathPlanner will not work.");
      e.printStackTrace();
    }
  }
    */

  /**
   * [NUEVO] Pone los módulos en X (X-Stance) para defensa. Hace muy difícil que empujen el robot.
   */
  public void lock() {
    SwerveModuleState[] states =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)), // FL
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), // FR
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), // BL
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)) // BR
        };
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(states[i]);
    }
  }

  /**
   * [NUEVO] Método para inyectar datos de Visión (PhotonVision/Limelight). Los equipos PowerHouse
   * mezclan odometría + visión aquí.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void zeroGyro() {
    gyroIO.reset();
    Pose2d currentPose = getPose();
    resetPose(new Pose2d(currentPose.getTranslation(), new Rotation2d()));
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
        Rotation2d.fromRadians(gyroInputs.yawPositionRad), getModulePositions(), pose);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** [OPTIMIZADO] Retorna posiciones reusando el array de memoria. */
  public SwerveModulePosition[] getModulePositions() {
    for (int i = 0; i < 4; i++) {
      modules[i].updatePosition(modulePositions[i]);
    }
    return modulePositions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }
}
