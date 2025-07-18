package frc.lib.interfaces.led;

public record TimedColor(Color color, double intervalSec) {
  @Override
  public String toString() {
    return "(" + color.r() + "," + color.g() + "," + color.b() + "," + intervalSec + ")";
  }
}
