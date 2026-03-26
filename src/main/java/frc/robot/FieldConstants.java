package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.LoggedTunableNumber;

public final class FieldConstants {
  public static final double FIELD_LENGTH_BASE = 17.548;
  public static final double FIELD_WIDTH = 8.052;

  public static final LoggedTunableNumber fieldLengthCorrection =
      new LoggedTunableNumber("Tuning/Field/LengthCorrection", 0.0);

  public static double getFieldLength() {
    return FIELD_LENGTH_BASE + fieldLengthCorrection.get();
  }

  public static final Pose2d BLUE_SHOOTING_LEFT = new Pose2d(4.0, 7.6, Rotation2d.fromDegrees(90));

  // --- FIELD BLUE ---

  // 1. HUB (Centro del campo aprox para el ejemplo)
  public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.625, 4.04);

  // 2. ALLIANCE WALL / FEEDER (Para mirar a 180° hacia "atrás")
  // Definimos un punto virtual muy lejos detrás de la pared de drivers azul.
  // Al apuntar aquí, el robot siempre mirará hacia su propia alianza (180
  // grados).
  public static final Translation2d BLUE_ALLIANCE_WALL_TARGET = new Translation2d(-10.0, 4.0);

  // --- FLIP LOGIC ---

  public static Pose2d onAlliance(Pose2d bluePose) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return new Pose2d(
          new Translation2d(getFieldLength() - bluePose.getX(), bluePose.getY()),
          new Rotation2d(Math.PI).minus(bluePose.getRotation()));
    }
    return bluePose;
  }

  /** Sobrecarga para Translation2d (Puntos sin rotación). Esencial para el Snap-To-Target. */
  public static Translation2d onAlliance(Translation2d blueTranslation) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return new Translation2d(getFieldLength() - blueTranslation.getX(), blueTranslation.getY());
    }
    return blueTranslation;
  }
}
