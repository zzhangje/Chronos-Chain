package frc.lib.interfaces.sensor.digital;

public class DigitalInputRio implements DigitalInput {
  private final edu.wpi.first.wpilibj.DigitalInput input;

  public DigitalInputRio(int channel) {
    this.input = new edu.wpi.first.wpilibj.DigitalInput(channel);
  }

  @Override
  public boolean getAsBoolean() {
    return input.get();
  }
}
