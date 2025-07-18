package frc.lib.interfaces.sensor.digital;

import java.util.function.BooleanSupplier;

public interface DigitalInput extends BooleanSupplier {
  @Override
  default boolean getAsBoolean() {
    return false;
  }
}
