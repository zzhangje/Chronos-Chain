package frc.lib.interfaces.sensor.digital;

public interface BiDigitalIO {
  default boolean getValue1() {
    return false;
  }

  default boolean getValue2() {
    return false;
  }

  default DigitalIO getDigitalIO1() {
    return new DigitalIO() {};
  }

  default DigitalIO getDigitalIO2() {
    return new DigitalIO() {};
  }
}
