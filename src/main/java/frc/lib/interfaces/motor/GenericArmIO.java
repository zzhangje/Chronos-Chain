package frc.lib.interfaces.motor;

import org.littletonrobotics.junction.AutoLog;

public interface GenericArmIO {
  @AutoLog
  class GenericArmIOInputs {
    public boolean connected;

    public double velRadPerSec;
    public double positionRad;
    public double outputVoltageVolt;
    public double supplyCurrentAmp;
    public double statorCurrentAmp;
  }

  default void updateInputs(GenericArmIOInputs inputs) {}

  default void setPosition(double positionRad, double feedforward) {}

  default void setPosition(
      double positionRad, double velRadPerSec, double accelRadPerSec2, double feedforward) {}

  default void setPdf(double kp, double kd, double ks, double kg) {}

  default void setCurrent(double currentAmp) {}

  default void setVoltage(double voltageVolt) {}

  default void stop() {}

  default void home(double homeAngleRad) {}
}
