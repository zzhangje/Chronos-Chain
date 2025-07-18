package frc.lib.interfaces.sensor.digital;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalIORio implements DigitalIO {
  private final DigitalInput input;

  public DigitalIORio(int channel) {
    this.input = new DigitalInput(channel);
  }

  @Override
  public boolean getAsBoolean() {
    return input.get();
  }
}
