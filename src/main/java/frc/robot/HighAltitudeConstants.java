// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.LoggedTunableNumber;

public final class HighAltitudeConstants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  // =============================================================================
  // GLOBAL
  // =============================================================================
  public static final double LOOP_PERIOD_SECS = 0.02;
  public static final double MAX_VOLTAGE = 12.0;

  // =============================================================================
  // CONTROLS (NUEVO)
  // =============================================================================
  public static final class Controls {
    public static final double DEADBAND = 0.1;
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
  }

  // --- GLOBAL FLAGS ---
  // Poner en FALSE para competencia real (seguridad).
  // Poner en TRUE para habilitar el Tuning en Dashboard.
  public static final boolean TUNING_MODE = false;

  // ... (Otras constantes existentes del Swerve) ...

  public static final class Auto {

    // --- PATHPLANNER PIDS (Suaves para Trayectorias) ---
    // Separados del Snap-To-Pose para evitar oscilaciones en movimiento
    public static final double PATHPLANNER_TRANSLATION_KP = 1.70;
    public static final double PATHPLANNER_TRANSLATION_KI = 0.001;
    public static final double PATHPLANNER_TRANSLATION_KD = 0.00;

    public static final double PATHPLANNER_ROTATION_KP = 3.5;
    public static final double PATHPLANNER_ROTATION_KI = 0.0;
    public static final double PATHPLANNER_ROTATION_KD = 0.0651;

    // Constraints Globales para Pathfinding (Teleop)
    public static final double PATHFINDING_VELOCITY = 3.0; // m/s (Más lento que Auto)
    public static final double PATHFINDING_ACCEL = 2.0; // m/s²

    // --- CONSTRAINTS FÍSICOS (Hardcoded - Rara vez se tunean en vivo) ---
    // Travel Mode (Rápido)
    public static final double TRAVEL_LINEAR_VELOCITY = 4.5;
    public static final double TRAVEL_LINEAR_ACCELERATION = 3.0;
    public static final double TRAVEL_ANGULAR_VELOCITY = Math.PI * 2; // ~360 deg/s
    public static final double TRAVEL_ANGULAR_ACCELERATION = Math.PI * 4;

    // Precision Mode (Lento)
    public static final double PRECISION_LINEAR_VELOCITY = 1.5;
    public static final double PRECISION_LINEAR_ACCELERATION = 1.5;
    public static final double PRECISION_ANGULAR_VELOCITY = Math.PI; // ~180 deg/s
    public static final double PRECISION_ANGULAR_ACCELERATION = Math.PI * 2;

    public static final double POSE_TOLERANCE_METERS = 0.05;
    public static final double POSE_TOLERANCE_RADIANS = Units.degreesToRadians(2.0);

    // --- LIVE TUNABLE PIDs (Dual Gains) ---

    // 1. TRAVEL MODE (Movement)
    public static final LoggedTunableNumber travelDriveKp =
        new LoggedTunableNumber("Tuning/DriveToPose/Travel/Translation/kP", 2.5);
    public static final LoggedTunableNumber travelDriveKi =
        new LoggedTunableNumber("Tuning/DriveToPose/Travel/Translation/kI", 0.0);
    public static final LoggedTunableNumber travelDriveKd =
        new LoggedTunableNumber("Tuning/DriveToPose/Travel/Translation/kD", 0.0);

    public static final LoggedTunableNumber travelSteerKp =
        new LoggedTunableNumber("Tuning/DriveToPose/Travel/Rotation/kP", 3.5);
    public static final LoggedTunableNumber travelSteerKi =
        new LoggedTunableNumber("Tuning/DriveToPose/Travel/Rotation/kI", 0.0);
    public static final LoggedTunableNumber travelSteerKd =
        new LoggedTunableNumber("Tuning/DriveToPose/Travel/Rotation/kD", 0.05);

    // 2. PRECISION MODE (Scoring/Climbing)
    public static final LoggedTunableNumber precisionDriveKp =
        new LoggedTunableNumber("Tuning/DriveToPose/Precision/Translation/kP", 4.0);
    public static final LoggedTunableNumber precisionDriveKi =
        new LoggedTunableNumber("Tuning/DriveToPose/Precision/Translation/kI", 0.0);
    public static final LoggedTunableNumber precisionDriveKd =
        new LoggedTunableNumber("Tuning/DriveToPose/Precision/Translation/kD", 0.0);

    public static final LoggedTunableNumber precisionSteerKp =
        new LoggedTunableNumber("Tuning/DriveToPose/Precision/Rotation/kP", 3.0);
    public static final LoggedTunableNumber precisionSteerKi =
        new LoggedTunableNumber("Tuning/DriveToPose/Precision/Rotation/kI", 0.0);
    public static final LoggedTunableNumber precisionSteerKd =
        new LoggedTunableNumber("Tuning/DriveToPose/Precision/Rotation/kD", 0.0);
  }

  // --- PHYSICS SIMULATION CONSTANTS ---
  // Masa del robot (aprox 90lbs)
  // --- FÍSICA DEL ROBOT (Full Physics para Choreo) ---
  public static final double MASS_KG = 53.5; // Estimado (ajustar con báscula)
  public static final double MOI_KG_M2 = 2.74; // Inercia rotacional estimada
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

  // --- DIMENSIONES DE COLISIÓN (Safety First - Hopper Expandido) ---
  // Aunque el Frame es mas chico, le decimos a PathPlanner que somos así de
  // grandes
  public static final double ROBOT_COLLISION_LENGTH = Units.inchesToMeters(39.75); // ~1.01m
  public static final double ROBOT_COLLISION_WIDTH = Units.inchesToMeters(37.5); // ~0.95m
  // Momentos de Inercia (J)
  // Inercia del mecanismo de giro (masa pequeña rotando)
  public static final double MOI_TURN_KG_M2 = 0.004;
  // Inercia de la transmisión (rueda + carga efectiva del robot sobre la rueda)
  public static final double MOI_DRIVE_KG_M2 = 0.025;

  // =============================================================================
  // SWERVE SUBSYSTEM
  // =============================================================================
  public static final class Swerve {
    // --- Physical Dimensions ---
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(19.75);
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(23.75);
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    // --- Gearing & Conversions ---
    // Drive: (50/16) * (16/28) * (45/15) approx 5.357
    public static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
    // Turn: MK4i Standard (150/7) approx 21.428
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    // Conversion Factors
    public static final double DRIVE_METERS_PER_ROTATION =
        (2.0 * Math.PI * WHEEL_RADIUS_METERS) / DRIVE_GEAR_RATIO;
    public static final double TURN_RADIANS_PER_ROTATION = (2.0 * Math.PI) / TURN_GEAR_RATIO;

    // --- Kinematics ---
    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0), // FL
            new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0), // FR
            new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0), // BL
            new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0) // BR
            );

    // --- DRIVE CONTROL (PID + FF) ---
    public static final double DRIVE_KP = 0.320;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KS = 0.2;
    public static final double DRIVE_KV = 2.183;

    // --- TURN CONTROL (Profiled PID) ---
    public static final double TURN_KP = 5.0; // /5
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 0.0;

    // Constraints for ProfiledPIDController
    public static final double TURN_MAX_VELOCITY_RAD_S = 6.0;
    public static final double TURN_MAX_ACCELERATION_RAD_S2 = 15.0;

    // --- Limits ---
    public static final double MAX_LINEAR_SPEED_M_S = 5.0;
    public static final double MAX_ANGULAR_SPEED_RAD_S = 6.28;

    // NUEVO: Agregado para SlewRateLimiter en SwerveModule
    public static final double MAX_ACCEL_M_S2 = 15.0;

    // --- Module Configuration ---
    public record ModuleConstants(
        int driveID, int turnID, int cancoderID, Rotation2d offset, boolean driveInverted) {}

    public static final ModuleConstants MOD_FL =
        new ModuleConstants(14, 21, 34, Rotation2d.fromRotations(0.030), false);
    public static final ModuleConstants MOD_FR =
        new ModuleConstants(13, 22, 33, Rotation2d.fromRotations(0.387), true);
    public static final ModuleConstants MOD_BL =
        new ModuleConstants(12, 24, 32, Rotation2d.fromRotations(-0.124), false);
    public static final ModuleConstants MOD_BR =
        new ModuleConstants(11, 23, 31, Rotation2d.fromRotations(-0.216), true);
  }

  // Shooter
  public static final class Shooter {
    public static final double MAX_FLYWHEEL_SPEED_RADS_PER_SEC = 380; // 460 was working
    // 450

    public static final double KP = 0.7;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.20;

    public static final double FLYWHEEL_RADIUS = 0.0508;
    public static final double SLIP_FACTOR = 0.7;

    // Motor Ports
    public static final int SHOOTER_RIGHT_MOTOR_ID = 61;
    public static final int SHOOTER_LEFT_MOTOR_ID = 62;

    // PD Control
    public static final double HOOD_kP = 15.0;
    public static final double HOOD_kD = 0.0;

    public static final int HOOD_MOTOR_ID = 60;

    // Constantes de límites
    public static final double HOOD_MIN_ANGLE_RAD = 0.0;
    public static final double HOOD_MAX_ANGLE_RAD =
        Math.toRadians(45); // Valor temporal (45 grados)

    // target autoaim
    public static final double GOAL_HEIGHT = 1.9; // meters
    public static final double SHOOTER_HEIGHT = 0.428752; // meters

    public static final double SHOOTER_OFFSET = 50;
  }

  // Pivot
  public static final class Pivot {
    public static final double GEAR_RATIO = 45.83;
    public static final double RETRACT_POS = 0;
    public static final double EXTENDED_POS = 0.26;

    public static final int PIVOT_MOTOR_ID = 41;

    public static final double WOBBLE_HIGH = 0.10;
    public static final double WOBBLE_LOW = 0.26;
    public static final double WOBBLE_TOLERANCE = 0.1;
  }

  // Indexer
  public static final class Indexer {
    public static final double INDEXER_VOLTAGE = 0.6 * 12;
    public static final int INDEXER_MOTOR_ID = 51;

    public static final double STALL_CURRENT_AMPS = 40;
    public static final double EJECT_DURATION_SECONDS = 0.3;
  }

  // Intake
  public static final class Intake {
    public static final double INTAKE_VOLTAGE = 0.3 * 12;
    public static final int INTAKE_MOTOR_ID = 42;
  }

  // Feeder
  public static final class Feeder {
    public static final double FEEDER_VOLTAGE = 0.5 * 12;
    public static final int FEEDER_MOTOR_ID = 52;
  }

  // Leds
  public static final class Leds {
    public static final int LED_CANDLE_ID = 19;

    public static final int LED_COUNT = 23;

    public static final double LED_BRIGHTNESS = 0.41; // 0.67

    public static final double LED_BLINK_PERIOD = 0.25;
    public static final double LED_RAINBOW_SPEED = 0.02;

    public static final int RED_R = 255;
    public static final int RED_G = 0;
    public static final int RED_B = 0;

    public static final int GREEN_R = 0;
    public static final int GREEN_G = 255;
    public static final int GREEN_B = 0;

    public static final int BLUE_R = 0;
    public static final int BLUE_G = 0;
    public static final int BLUE_B = 255;

    public static final int HAR_R = 34;
    public static final int HAR_G = 191;
    public static final int HAR_B = 144;

    public static final int OFF_R = 0;
    public static final int OFF_G = 0;
    public static final int OFF_B = 0;
  }
}
