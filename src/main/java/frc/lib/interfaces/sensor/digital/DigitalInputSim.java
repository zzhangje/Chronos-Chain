package frc.lib.interfaces.sensor.digital;

import java.util.function.BooleanSupplier;

public class DigitalInputSim implements DigitalInput {
  private final BooleanSupplier valueSupplier;

  public DigitalInputSim(BooleanSupplier valueSupplier) {
    this.valueSupplier = valueSupplier;
  }

  @Override
  public boolean getAsBoolean() {
    return valueSupplier.getAsBoolean();
  }
}
