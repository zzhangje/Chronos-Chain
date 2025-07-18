package frc.robot;

import frc.lib.utils.GamePieceTracker;
import frc.reefscape.GamePiece;
import frc.reefscape.GamePiece.GamePieceType;
import lombok.Getter;
import lombok.Setter;

public class RobotState {
  @Getter
  private static final GamePieceTracker coral =
      new GamePieceTracker(GamePiece.GamePieceType.CORAL.getName(), 0.5);

  @Getter private static final Odometry odometry = new Odometry();

  public static class RobotGoal {
    @Getter private final GamePieceType selectedType;
    @Getter private final String selectedBranch;
    @Getter private final String selectedLevel;
    @Getter @Setter private Boolean ignoreArmMoveCondition;

    public RobotGoal(
        GamePieceType type, String branch, String level, Boolean ignoreArmMoveCondition) {
      this.selectedType = type;
      this.selectedBranch = branch;
      this.selectedLevel = level;
      this.ignoreArmMoveCondition = ignoreArmMoveCondition;
    }

    public static RobotGoal scoreNet() {
      return new RobotGoal(GamePieceType.ALGAE, "N", "0", false);
    }

    public static RobotGoal scoreProcessor() {
      return new RobotGoal(GamePieceType.ALGAE, "P", "0", false);
    }

    public static RobotGoal collectAlgae(String branch) {
      return new RobotGoal(GamePieceType.ALGAE, branch, "0", false);
    }

    public static RobotGoal scoreCoral(String branch, String level) {
      return new RobotGoal(GamePieceType.CORAL, branch, level, false);
    }

    public static RobotGoal invalid() {
      return new RobotGoal(GamePieceType.INVALID, "NA", "NA", false);
    }

    public boolean isValid() {
      return !selectedType.equals(GamePieceType.INVALID)
          && !selectedBranch.equals("NA")
          && !selectedLevel.equals("NA");
    }
  }
}
