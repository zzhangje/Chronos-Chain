package frc.lib.interfaces.motor;

public class SlamArmIOSim implements SlamArmIO {
  private final double minAngleRad;
  private final double maxAngleRad;
  private final boolean isInvert;

  private double targetAngleRad;

  public SlamArmIOSim(
      double minAngleRad, double maxAngleRad, double startingAngleRad, boolean isInvert) {
    this.minAngleRad = minAngleRad;
    this.maxAngleRad = maxAngleRad;
    this.isInvert = isInvert;

    targetAngleRad = startingAngleRad;
  }

  @Override
  public void updateInputs(SlamArmIOInputs inputs) {
    inputs.connected = true;
    inputs.positionRad = targetAngleRad;
  }

  @Override
  public void setCurrent(double currentAmp) {
    if (currentAmp > 0.0) {
      targetAngleRad = isInvert ? maxAngleRad : minAngleRad;
    } else {
      targetAngleRad = isInvert ? minAngleRad : maxAngleRad;
    }
  }
}
