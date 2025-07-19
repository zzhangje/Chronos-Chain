package frc.robot;

import frc.lib.utils.GamePieceTracker;
import frc.reefscape.GamePiece;
import frc.reefscape.GamePiece.GamePieceType;
import lombok.Getter;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class RobotState {
  @Getter
  private static final GamePieceTracker coralTracker =
      new GamePieceTracker(GamePiece.GamePieceType.CORAL.getName(), 0.5);

  @Getter private static final Odometry odometry = new Odometry();

  public static class RobotGoal implements LoggableInputs {
    @Getter private GamePieceType selectedType;
    @Getter private String selectedBranch;
    @Getter private String selectedLevel;
    @Getter private Boolean ignoreArmMoveCondition;

    public RobotGoal(
        GamePieceType type, String branch, String level, Boolean ignoreArmMoveCondition) {
      this.selectedType = type;
      this.selectedBranch = branch;
      this.selectedLevel = level;
      this.ignoreArmMoveCondition = ignoreArmMoveCondition;
    }

    public RobotGoal setIgnoreArmMoveCondition(Boolean ignoreArmMoveCondition) {
      return new RobotGoal(
          this.selectedType, this.selectedBranch, this.selectedLevel, ignoreArmMoveCondition);
    }

    public static RobotGoal scoreNet() {
      return new RobotGoal(GamePieceType.ALGAE, "N", "0", false);
    }

    public Boolean isScoreNet() {
      return selectedType == GamePieceType.ALGAE && selectedBranch.equals("N");
    }

    public static RobotGoal scoreProcessor() {
      return new RobotGoal(GamePieceType.ALGAE, "P", "0", false);
    }

    public Boolean isScoreProcessor() {
      return selectedType == GamePieceType.ALGAE && selectedBranch.equals("P");
    }

    public static RobotGoal collectAlgae(String branch) {
      return new RobotGoal(GamePieceType.ALGAE, branch, "0", false);
    }

    public Boolean isHighPick() {
      return selectedBranch.equals("AB")
          || selectedBranch.equals("EF")
          || selectedBranch.equals("IJ");
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

    @Override
    public void toLog(LogTable table) {
      table.put("selectedType", selectedType.getName());
      table.put("selectedBranch", selectedBranch);
      table.put("selectedLevel", selectedLevel);
      table.put("ignoreArmMoveCondition", ignoreArmMoveCondition);
    }

    @Override
    public void fromLog(LogTable table) {
      selectedType = GamePieceType.fromString(table.get("selectedType", "INVALID"));
      selectedBranch = table.get("selectedBranch", "NA");
      selectedLevel = table.get("selectedLevel", "NA");
      ignoreArmMoveCondition = table.get("ignoreArmMoveCondition", false);
    }
  }
}
