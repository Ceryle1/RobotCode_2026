package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

public class LedsIOCANdle implements LedsIO {
  private final CANdle candle;
  private final int ledCount;

  public LedsIOCANdle(int canId, int ledCount) {
    this.ledCount = ledCount;
    this.candle = new CANdle(canId);
  }

  @Override
  public void applyOutputs(LedsIOOutputs outputs) {
    if (outputs.buffer == null || outputs.buffer.length < 3) {
      candle.setControl(new SolidColor(8, 8 + ledCount - 1).withColor(new RGBWColor(0, 0, 0)));
      return;
    }

    int maxLedsFromBuffer = outputs.buffer.length / 3;
    int ledsToWrite = Math.min(ledCount, maxLedsFromBuffer);

    for (int i = 0; i < ledsToWrite; i++) {
      int r = outputs.buffer[i * 3] & 0xFF;
      int g = outputs.buffer[i * 3 + 1] & 0xFF;
      int b = outputs.buffer[i * 3 + 2] & 0xFF;

      candle.setControl(new SolidColor(8 + i, 8 + i).withColor(new RGBWColor(r, g, b)));
    }
  }
}
