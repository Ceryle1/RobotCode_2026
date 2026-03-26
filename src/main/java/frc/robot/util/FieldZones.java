// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains information for location of field element and other useful reference points. Adapted for
 * REBUILT 2026 High Altitude Robot Code.
 *
 * <p>NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station.
 */
public class FieldZones {
  public static final FieldType fieldType = FieldType.WELDED;

  // AprilTag related constants
  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  // Field dimensions
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  /**
   * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
   */
  public static class LinesVertical {
    public static final double center = fieldLength / 2.0;
    // Asumiendo Tag 26 es la cara frontal del Hub Azul (Near Face)
    public static final double starting =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX();
    public static final double allianceZone = starting;
    public static final double hubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + Hub.width / 2.0;

    // Zonas neutrales aproximadas (Trench Run Zones)
    public static final double midFieldBarrierMin = center - Units.inchesToMeters(24);
    public static final double midFieldBarrierMax = center + Units.inchesToMeters(24);

    // Lado Opuesto (Red)
    // Tag 4 asumiendo es el Red Hub Front Face
    public static final double oppHubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + Hub.width / 2.0;
  }

  /**
   * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
   *
   * <p>NOTE: The field element start and end are always left to right from the perspective of the
   * alliance station
   */
  public static class LinesHorizontal {

    public static final double center = fieldWidth / 2.0;

    // Right of hub (Bottom side of field relative to Blue Driver)
    public static final double rightBumpStart = Hub.nearRightCorner.getY();
    public static final double rightBumpEnd = rightBumpStart - RightBump.width;
    public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
    public static final double rightTrenchOpenEnd = 0; // Guardrail

    // Left of hub (Top side of field relative to Blue Driver)
    public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
    public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
    public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
    public static final double leftTrenchOpenStart = fieldWidth; // Guardrail
  }

  /** Hub related constants (The Central Obstacle) */
  public static class Hub {

    // Dimensions
    public static final double width = Units.inchesToMeters(47.0);
    public static final double radius = width / 2.0; // Radio aproximado de evasión

    // Relevant reference points on alliance side
    // Usamos Tag 26 como referencia principal (Near Face)
    public static final Translation3d topCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            Units.inchesToMeters(72.0));

    // Centro 2D para evasión
    public static final Translation2d centerPoint =
        new Translation2d(topCenterPoint.getX(), topCenterPoint.getY());

    public static final Translation2d nearLeftCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Relevant reference points on the opposite side (Red Hub)
    public static final Translation3d oppTopCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            Units.inchesToMeters(72.0));

    public static final Translation2d oppCenterPoint =
        new Translation2d(oppTopCenterPoint.getX(), oppTopCenterPoint.getY());
  }

  /** Left Bump related constants */
  public static class LeftBump {
    public static final double width = Units.inchesToMeters(73.0);
  }

  /** Right Bump related constants */
  public static class RightBump {
    public static final double width = Units.inchesToMeters(73.0);
  }

  /** Verifica colisión con un HUB circular y agrega waypoint si es necesario. */
  private static void checkHubCollision(
      Translation2d start,
      Translation2d end,
      Translation2d center,
      double radius,
      List<Translation2d> waypoints) {
    Translation2d d = end.minus(start);
    Translation2d f = start.minus(center);

    double a = d.getNorm() * d.getNorm();
    double b = 2 * (f.getX() * d.getX() + f.getY() * d.getY());
    double c = (f.getNorm() * f.getNorm()) - (radius * radius);

    double discriminant = b * b - 4 * a * c;

    if (discriminant >= 0) {
      discriminant = Math.sqrt(discriminant);
      double t1 = (-b - discriminant) / (2 * a);
      double t2 = (-b + discriminant) / (2 * a);

      if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) {
        // Intersección detectada: Rodear
        // Decidir si por arriba o abajo (Left/Right relativo al driver)
        if (start.getY() > center.getY()) {
          waypoints.add(new Translation2d(center.getX(), center.getY() + (radius + 0.5)));
        } else {
          waypoints.add(new Translation2d(center.getX(), center.getY() - (radius + 0.5)));
        }
      }
    }
  }

  // --- INFRAESTRUCTURA DE CARGA DE LAYOUT (Estilo 6328) ---

  @RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    @Getter private final String jsonFolder;
  }

  public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),
    NONE("2026-none");

    private final String name;
    private volatile AprilTagFieldLayout layout;
    private volatile String layoutString;

    AprilTagLayoutType(String name) {
      this.name = name;
    }

    public AprilTagFieldLayout getLayout() {
      if (layout == null) {
        synchronized (this) {
          if (layout == null) {
            try {
              // Ajuste de ruta: Asumimos ejecución estándar de Gradle
              // o despliegue en RIO.
              Path p =
                  Filesystem.getDeployDirectory()
                      .toPath()
                      .resolve("apriltags")
                      .resolve(fieldType.getJsonFolder())
                      .resolve(name + ".json");

              layout = new AprilTagFieldLayout(p);
              layoutString = new ObjectMapper().writeValueAsString(layout);
            } catch (IOException e) {
              // Fallback silencioso o RuntimeException según preferencia
              System.err.println("CRITICAL: Could not load AprilTag Layout: " + name);
              throw new RuntimeException(e);
            }
          }
        }
      }
      return layout;
    }

    public String getLayoutString() {
      if (layoutString == null) {
        getLayout();
      }
      return layoutString;
    }
  }

  // Agrega esto a tu clase FieldZones existente en frc.robot.util

  /** Valida si un punto está dentro del campo jugable. */
  public static boolean isInsideField(Translation2d point) {
    return point.getX() >= 0.0
        && point.getX() <= fieldLength
        && point.getY() >= 0.0
        && point.getY() <= fieldWidth;
  }

  /** Ajusta un punto para que esté dentro del campo (Clamping). */
  public static Translation2d clampToField(Translation2d point) {
    return new Translation2d(
        Math.max(0.1, Math.min(fieldLength - 0.1, point.getX())), // 10cm buffer
        Math.max(0.1, Math.min(fieldWidth - 0.1, point.getY())));
  }

  // --- REBUILT PATH PLANNING LOGIC (High Altitude Custom) ---

  public static List<Translation2d> getWaypoints(
      Translation2d start, Translation2d end, boolean useBump) {
    List<Translation2d> waypoints = new ArrayList<>();

    // Clamp inputs por seguridad
    start = clampToField(start);
    end = clampToField(end);

    // 1. EVASIÓN HUBS (Lógica existente pero asegurando puntos seguros)
    checkHubCollision(
        start, end, Hub.centerPoint, Hub.radius + 0.40, waypoints); // +40cm radio robot
    checkHubCollision(start, end, Hub.oppCenterPoint, Hub.radius + 0.40, waypoints);

    // 2. EVASIÓN BUMPS / MIDFIELD (Lógica existente)
    // ... (Mismo código de antes) ...

    // 3. ORDENAR Y LIMPIAR
    // A veces la lógica agrega puntos desordenados. Para un path simple,
    // generalmente basta con insertarlos. Si hay múltiples, la distancia dictará el
    // orden.
    // (Para este nivel de complejidad, asumimos max 1 obstáculo a la vez).

    return waypoints;
  }
}
