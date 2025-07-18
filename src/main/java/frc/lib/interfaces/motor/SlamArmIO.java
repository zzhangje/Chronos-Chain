package frc.lib.interfaces.motor;

import org.littletonrobotics.junction.AutoLog;

public interface SlamArmIO {
  @AutoLog
  class SlamArmIOInputs {
    public boolean connected;

    public double velRadPerSec;
    public double positionRad;
    public double outputVoltageVolt;
    public double supplyCurrentAmp;
    public double statorCurrentAmp;
  }

  default void updateInputs(SlamArmIOInputs inputs) {}

  default void setCurrent(double currentAmp) {}

  default void stop() {}
}
