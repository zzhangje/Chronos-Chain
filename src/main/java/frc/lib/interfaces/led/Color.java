package frc.lib.interfaces.led;

public record Color(int r, int g, int b) {
  public static final Color RED = new Color(255, 0, 0);
  public static final Color BLUE = new Color(0, 0, 255);
  public static final Color ALGAE_BLUE = new Color(106, 210, 235);
  public static final Color GREEN = new Color(0, 255, 0);
  public static final Color YELLOW = new Color(255, 215, 0);
  public static final Color ORANGE = new Color(255, 155, 0);
  public static final Color PURPLE = new Color(255, 0, 255);
  public static final Color WHITE = new Color(255, 255, 255);
  public static final Color OFF = new Color(0, 0, 0);

  @Override
  public String toString() {
    return "(" + r + "," + g + "," + b + ")";
  }
}
