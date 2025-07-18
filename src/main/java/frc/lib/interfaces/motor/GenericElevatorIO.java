package frc.lib.interfaces.motor;

import org.littletonrobotics.junction.AutoLog;

public interface GenericElevatorIO {
  @AutoLog
  class GenericElevatorIOInputs {
    public boolean connected;

    public double velMeterPerSec;
    public double positionMeter;
    public double outputVoltageVolt;
    public double supplyCurrentAmp;
    public double statorCurrentAmp;
  }

  default void updateInputs(GenericElevatorIOInputs inputs) {}

  default void setPosition(double positionMeter, double feedforward) {}

  default void setPosition(
      double positionMeter, double velMeterPerSec, double accelMeterPerSec2, double feedforward) {}

  default void setPdf(double kp, double kd, double ks, double kg) {}

  default void setCurrent(double currentAmp) {}

  default void stop() {}

  default void home(double homePositionMeter) {}
}
