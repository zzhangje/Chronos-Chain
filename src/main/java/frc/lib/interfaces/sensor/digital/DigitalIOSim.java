package frc.lib.interfaces.sensor.digital;

import java.util.function.BooleanSupplier;

public class DigitalIOSim implements DigitalIO {
  private final BooleanSupplier valueSupplier;

  public DigitalIOSim(BooleanSupplier valueSupplier) {
    this.valueSupplier = valueSupplier;
  }

  @Override
  public boolean getAsBoolean() {
    return valueSupplier.getAsBoolean();
  }
}
