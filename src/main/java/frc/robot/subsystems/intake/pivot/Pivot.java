package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.intake.pivot.PivotIO.PivotIOOutputs;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final PivotIOOutputs outputs = new PivotIOOutputs();

  private static final double PIVOT_RETRACTED_ROT =
      HighAltitudeConstants.Pivot.RETRACT_POS * HighAltitudeConstants.Pivot.GEAR_RATIO;
  private static final double PIVOT_EXPANDED_ROT =
      HighAltitudeConstants.Pivot.EXTENDED_POS * HighAltitudeConstants.Pivot.GEAR_RATIO;

  public Pivot(PivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    io.applyOutputs(outputs);
    Logger.recordOutput("Pivot/TargetPositionRot", outputs.targetPositionRot);
  }

  // API

  public void retract() {
    outputs.targetPositionRot = PIVOT_RETRACTED_ROT;
  }

  public void expand() {
    outputs.targetPositionRot = PIVOT_EXPANDED_ROT;
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public void setTarget(double motorRotations) {
    outputs.targetPositionRot = motorRotations;
  }

  // Getters

  public double getPositionRot() {
    return inputs.positionRot;
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  public boolean atSetpoint(double toleranceRot) {
    return Math.abs(inputs.positionRot - outputs.targetPositionRot) < toleranceRot;
  }
}
