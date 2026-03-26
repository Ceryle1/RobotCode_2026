package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HighAltitudeConstants;
import frc.robot.subsystems.leds.LedsIO.LedsIOOutputs;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {
  private final LedsIO io;
  private final LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();
  private final LedsIOOutputs outputs = new LedsIOOutputs();

  public Leds(LedsIO io) {
    this.io = io;
    outputs.buffer = new byte[HighAltitudeConstants.Leds.LED_COUNT * 3];
    off();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Leds", inputs);

    io.applyOutputs(outputs);
    Logger.recordOutput("Leds/BufferLength", outputs.buffer.length);
  }

  public void setSolid(int r, int g, int b) {
    for (int i = 0; i < HighAltitudeConstants.Leds.LED_COUNT; i++) {
      outputs.buffer[i * 3] = (byte) r;
      outputs.buffer[i * 3 + 1] = (byte) g;
      outputs.buffer[i * 3 + 2] = (byte) b;
    }
  }

  public void setLed(int index, int r, int g, int b) {
    if (index < 0 || index >= HighAltitudeConstants.Leds.LED_COUNT) {
      return;
    }

    outputs.buffer[index * 3] = (byte) r;
    outputs.buffer[index * 3 + 1] = (byte) g;
    outputs.buffer[index * 3 + 2] = (byte) b;
  }

  public void off() {
    setSolid(
        HighAltitudeConstants.Leds.OFF_R,
        HighAltitudeConstants.Leds.OFF_G,
        HighAltitudeConstants.Leds.OFF_B);
  }

  public void red() {
    setSolid(
        HighAltitudeConstants.Leds.RED_R,
        HighAltitudeConstants.Leds.RED_G,
        HighAltitudeConstants.Leds.RED_B);
  }

  public void green() {
    setSolid(
        HighAltitudeConstants.Leds.GREEN_R,
        HighAltitudeConstants.Leds.GREEN_G,
        HighAltitudeConstants.Leds.GREEN_B);
  }

  public void har() {
    setSolid(
        HighAltitudeConstants.Leds.HAR_R,
        HighAltitudeConstants.Leds.HAR_G,
        HighAltitudeConstants.Leds.HAR_B);
  }

  public void blue() {
    setSolid(
        HighAltitudeConstants.Leds.BLUE_R,
        HighAltitudeConstants.Leds.BLUE_G,
        HighAltitudeConstants.Leds.BLUE_B);
  }
}
