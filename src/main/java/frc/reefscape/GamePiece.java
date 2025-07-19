package frc.reefscape;

import edu.wpi.first.math.util.Units;
import lombok.Getter;

public class GamePiece {
  public static class Coral {
    public static final double LENGTH = Units.inchesToMeters(11.0 + 7.0 / 8.0);
    public static final double DIAMETER = Units.inchesToMeters(4.0);
    public static final int OBJ_DETECT_ID = 1;
  }

  public static class Algae {
    public static final double DIAMETER = Units.inchesToMeters(16.0);
    public static final int OBJ_DETECT_ID = 0;
  }

  @Getter
  public enum GamePieceType {
    INVALID("Invalid"),
    ALGAE("Algae"),
    CORAL("Coral");

    private final String name;

    GamePieceType(String name) {
      this.name = name;
    }

    public static GamePieceType fromString(String name) {
      for (GamePieceType type : values()) {
        if (type.name.equalsIgnoreCase(name)) {
          return type;
        }
      }
      return INVALID;
    }
  }
}
