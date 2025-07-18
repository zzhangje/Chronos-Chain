package frc.lib.interfaces.sensor.digital;

public interface BiDigitalInput {
  default boolean getValue1() {
    return false;
  }

  default boolean getValue2() {
    return false;
  }

  default DigitalInput getDigitalIO1() {
    return new DigitalInput() {};
  }

  default DigitalInput getDigitalIO2() {
    return new DigitalInput() {};
  }
}
