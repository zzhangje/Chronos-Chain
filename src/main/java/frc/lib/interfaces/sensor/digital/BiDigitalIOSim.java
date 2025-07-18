package frc.lib.interfaces.sensor.digital;

import java.util.function.BooleanSupplier;

public class BiDigitalIOSim implements BiDigitalIO {
  private final BooleanSupplier value1Supplier;
  private final BooleanSupplier value2Supplier;

  public BiDigitalIOSim(BooleanSupplier value1Supplier, BooleanSupplier value2Supplier) {
    this.value1Supplier = value1Supplier;
    this.value2Supplier = value2Supplier;
  }

  @Override
  public boolean getValue1() {
    return value1Supplier.getAsBoolean();
  }

  @Override
  public boolean getValue2() {
    return value2Supplier.getAsBoolean();
  }

  @Override
  public DigitalIO getDigitalIO1() {
    return new DigitalIOSim(value1Supplier);
  }

  @Override
  public DigitalIO getDigitalIO2() {
    return new DigitalIOSim(value2Supplier);
  }
}
