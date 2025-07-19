package frc.robot.subsystem.nodeselector;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.interfaces.VirtualSubsystem;
import frc.lib.utils.AllianceFlipUtil;
import frc.reefscape.Field;
import frc.reefscape.GamePiece.GamePieceType;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotGoal;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;
import java.nio.file.Paths;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class NodeSelector extends VirtualSubsystem {
  private final StringPublisher nodePublisher;
  private final StringSubscriber nodeSubscriber;
  private final BooleanSubscriber isIgnoreArmMoveConditionSubscriber;
  private final IntegerPublisher timePublisher;
  private final BooleanPublisher isAutoPublisher;
  private final DoublePublisher reef2RobotAngleDegreePublisher;
  private final DoublePublisher reef2RobotDistanceMeterPublisher;

  @Getter private RobotGoal selectedNode = RobotGoal.invalid();

  public NodeSelector() {
    System.out.println("[NodeSelector] Starting server...");

    var table = NetworkTableInstance.getDefault().getTable("nodeselector");
    nodePublisher = table.getStringTopic("node_robot_2_dashboard").publish();
    nodeSubscriber = table.getStringTopic("node_dashboard_2_robot").subscribe("");
    isIgnoreArmMoveConditionSubscriber =
        table.getBooleanTopic("is_ignore_arm_move_condition").subscribe(false);
    timePublisher = table.getIntegerTopic("match_time").publish();
    isAutoPublisher = table.getBooleanTopic("is_auto").publish();
    reef2RobotAngleDegreePublisher = table.getDoubleTopic("reef_2_robot_angle_degree").publish();
    reef2RobotDistanceMeterPublisher =
        table.getDoubleTopic("reef_2_robot_distance_meter").publish();

    // Start server
    var app =
        Javalin.create(
            config ->
                config.staticFiles.add(
                    Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "nodeselector")
                        .toString(),
                    Location.EXTERNAL));
    app.start(5800);
  }

  @Override
  public void periodic() {
    timePublisher.set((long) Math.ceil(Math.max(0.0, DriverStation.getMatchTime())));
    isAutoPublisher.set(DriverStation.isAutonomous());

    var blueSideRobotPose = AllianceFlipUtil.apply(RobotState.getOdometry().getEstimatedPose());
    var fieldCentricReef2Robot = blueSideRobotPose.getTranslation().minus(Field.Reef.CENTER);
    var fieldCentricReef2RobotAngleDegree = fieldCentricReef2Robot.getAngle().getDegrees();
    var reef2RobotDistanceMeter =
        Math.max(fieldCentricReef2Robot.getNorm() - Field.Reef.CENTER_2_SIDE_DISTANCE, 0.0);

    reef2RobotAngleDegreePublisher.set(fieldCentricReef2RobotAngleDegree);
    reef2RobotDistanceMeterPublisher.set(reef2RobotDistanceMeter);

    Logger.recordOutput("NodeSelector/Reef2RobotAngleDegree", fieldCentricReef2RobotAngleDegree);
    Logger.recordOutput("NodeSelector/Reef2RobotDistanceMeter", reef2RobotDistanceMeter);

    Boolean isIgnoreArmMoveCondition = false;
    for (var val : isIgnoreArmMoveConditionSubscriber.readQueueValues()) {
      isIgnoreArmMoveCondition = val;
      selectedNode = selectedNode.setIgnoreArmMoveCondition(isIgnoreArmMoveCondition);
    }

    var selectedNodeStr = "";
    for (var val : nodeSubscriber.readQueueValues()) {
      selectedNodeStr = val;
    }

    if (selectedNodeStr.length() < 2) {
      return;
    }

    GamePieceType type;
    String branch;
    String level;

    if (selectedNodeStr.equals("AN")) {
      // Net selection
      type = GamePieceType.ALGAE;
      branch = "N";
      level = "0";
    } else if (selectedNodeStr.startsWith("A") && selectedNodeStr.length() == 3) {
      // Algae selection (e.g., AAB, ACD, AEF, AGH, AIJ, AKL)
      type = GamePieceType.ALGAE;
      branch = selectedNodeStr.substring(1); // Extract the 2-character branch (e.g., "AB", "CD")
      level = "0";
    } else if (selectedNodeStr.startsWith("C") && selectedNodeStr.length() == 3) {
      // Coral selection (e.g., CA1, CB2, CC3, CD4)
      type = GamePieceType.CORAL;
      branch =
          String.valueOf(
              selectedNodeStr.charAt(1)); // Extract single branch character (e.g., "A", "B")
      level = String.valueOf(selectedNodeStr.charAt(2)); // Extract level (e.g., "1", "2", "3", "4")
    } else {
      selectedNode = RobotGoal.invalid();
      return;
    }

    selectedNode = new RobotGoal(type, branch, level, isIgnoreArmMoveCondition);
    Logger.processInputs("NodeSelector/SelectedNode", selectedNode);
  }

  public void setSelected(RobotGoal goal) {
    this.selectedNode = goal;
    String nodeStr;

    if (goal.getSelectedType() == GamePieceType.ALGAE) {
      if (goal.getSelectedBranch().equals("N")) {
        // Net selection
        nodeStr = "AN";
      } else {
        // Algae selection (e.g., "AB" -> "AAB", "CD" -> "ACD")
        nodeStr = "A" + goal.getSelectedBranch();
      }
    } else {
      // Coral selection (e.g., branch="A", level="1" -> "CA1")
      nodeStr = "C" + goal.getSelectedBranch() + goal.getSelectedLevel();
    }

    nodePublisher.set(nodeStr);
  }

  public boolean isIgnoreArmMoveCondition() {
    return selectedNode.getIgnoreArmMoveCondition();
  }
}
