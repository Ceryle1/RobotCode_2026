package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class PhoenixOdometryThread extends Thread {
  private static PhoenixOdometryThread instance = null;

  private final List<StatusSignal<Angle>> driveSignals = new ArrayList<>();
  private final List<StatusSignal<Angle>> turnSignals = new ArrayList<>();
  private final Lock signalsLock = new ReentrantLock();

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
      instance.setName("PhoenixOdometryThread");
      instance.setDaemon(true);
      instance.start();
    }
    return instance;
  }

  public void registerSignals(StatusSignal<Angle> drivePos, StatusSignal<Angle> turnPos) {
    signalsLock.lock();
    try {
      driveSignals.add(drivePos);
      turnSignals.add(turnPos);
    } finally {
      signalsLock.unlock();
    }
  }

  @Override
  public void run() {
    while (true) {
      signalsLock.lock();
      try {
        if (driveSignals.isEmpty()) {
          continue; // Esperar hasta que se registren señales
        }

        // Esperar síncronamente SOLO a los Krakens (driveSignals) a 250Hz (0.004s)
        BaseStatusSignal.waitForAll(2.0 / 250.0, driveSignals.toArray(new BaseStatusSignal[0]));

        // Leer CANCoders asíncronamente (refresh false) para no saturar el bus CAN
        for (StatusSignal<Angle> turnSignal : turnSignals) {
          turnSignal.refresh(false).getValue();
        }

        // Aquí se deben poblar las colas (Queues) de AdvantageKit con los nuevos
        // timestamps y valores

      } finally {
        signalsLock.unlock();
      }
    }
  }
}
