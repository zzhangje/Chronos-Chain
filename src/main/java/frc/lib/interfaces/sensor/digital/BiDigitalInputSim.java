package frc.lib.interfaces.sensor.digital;

import java.util.function.BooleanSupplier;

public class BiDigitalInputSim implements BiDigitalInput {
  private final BooleanSupplier value1Supplier;
  private final BooleanSupplier value2Supplier;

  public BiDigitalInputSim(BooleanSupplier value1Supplier, BooleanSupplier value2Supplier) {
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
  public DigitalInput getDigitalIO1() {
    return new DigitalInputSim(value1Supplier);
  }

  @Override
  public DigitalInput getDigitalIO2() {
    return new DigitalInputSim(value2Supplier);
  }
}
