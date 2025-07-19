package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.reefscape.Field.Barge;
import frc.reefscape.Field.Processor;
import frc.reefscape.Field.Reef;
import frc.reefscape.GamePiece.GamePieceType;
import frc.robot.Constants.Misc;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotGoal;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;
import frc.robot.subsystem.arm.ArmGoal.EndEffectorGoal;
import frc.robot.subsystem.arm.command.SetArmGoalCommand;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.command.ProceedToProcessor;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class UniversalScoreCommand extends Command {
  private final Swerve swerve;
  private final Arm arm;
  private final Intake intake;
  private final RobotGoal goal;

  @AutoLogOutput(key = "Super/IsLeft")
  private final BooleanSupplier isLeftSupplier;

  private Command runningCommand;

  public UniversalScoreCommand(
      Swerve swerve,
      Arm arm,
      Intake intake,
      Supplier<RobotGoal> goal,
      BooleanSupplier isLeftSupplier) {
    this.swerve = swerve;
    this.arm = arm;
    this.intake = intake;
    this.goal = goal.get();
    this.isLeftSupplier = isLeftSupplier;
    addRequirements(swerve, arm, intake);
  }

  public UniversalScoreCommand(Swerve swerve, Arm arm, Intake intake, Supplier<RobotGoal> goal) {
    this(
        swerve,
        arm,
        intake,
        goal,
        () -> {
          if (goal.get().isScoreNet()) {
            return Barge.closestRobotSide(RobotState.getOdometry().getEstimatedPose());
          } else if (goal.get().isScoreProcessor()) {
            return Processor.closestRobotSide(RobotState.getOdometry().getEstimatedPose());
          } else {
            return Reef.closestRobotSide(RobotState.getOdometry().getEstimatedPose());
          }
        });
  }

  @Override
  public void initialize() {
    if (goal.getSelectedType() == GamePieceType.ALGAE) {
      // processor score
      if (goal.getSelectedBranch().equals("P")) {
        setName("Super/Score Processor");
        var goalPose = Processor.getScorePose();
        var driveCmd =
            new ProceedToProcessor(
                swerve,
                () -> goalPose.plus(isLeftSupplier.getAsBoolean() ? Misc.leftSide : Misc.rightSide),
                () -> goalPose.getRotation());
        runningCommand =
            Commands.parallel(
                Commands.either(driveCmd, Commands.none(), () -> !goal.getIgnoreArmMoveCondition()),
                Commands.sequence(
                    new SetArmGoalCommand(
                        arm,
                        () -> ArmSubsystemGoal.ALGAE_PROCESSOR_SCORE.setIsLeft(isLeftSupplier)),
                    Commands.waitUntil(() -> arm.stopAtGoal()),
                    Commands.runOnce(() -> arm.setEeGoal(EndEffectorGoal.ALGAE_SCORE)),
                    Commands.waitUntil(() -> !arm.hasAlgae()),
                    Commands.runOnce(() -> arm.setEeGoal(EndEffectorGoal.IDLE))));
      } else if (goal.getSelectedBranch().equals("N")) {
        setName("Super/Score Net");
        var goalPose = Barge.getAlgaeScoredPose(RobotState.getOdometry().getEstimatedPose());
        var driveCmd =
            new ProceedToProcessor(
                swerve,
                () -> goalPose.plus(isLeftSupplier.getAsBoolean() ? Misc.leftSide : Misc.rightSide),
                () -> goalPose.getRotation());
        runningCommand =
            Commands.parallel(
                Commands.either(driveCmd, Commands.none(), () -> !goal.getIgnoreArmMoveCondition()),
                Commands.sequence(
                    Commands.waitUntil(
                        () ->
                            goal.getIgnoreArmMoveCondition()
                                || (driveCmd.hasHeadingAtGoal()
                                    && driveCmd.hasDistanceWithin(1.2))),
                    new SetArmGoalCommand(
                        arm, () -> ArmSubsystemGoal.ALGAE_NET_SCORE.setIsLeft(isLeftSupplier)),
                    Commands.waitUntil(() -> arm.stopAtGoal()),
                    Commands.runOnce(() -> arm.setEeGoal(EndEffectorGoal.ALGAE_SCORE)),
                    Commands.waitUntil(() -> !arm.hasAlgae()),
                    Commands.runOnce(() -> arm.setEeGoal(EndEffectorGoal.IDLE))));
      } else {
      }
    } else if (goal.getSelectedType() == GamePieceType.CORAL) {
      if (goal.getSelectedLevel() == "1") {
      } else {
      }
    } else {
      runningCommand = Commands.none();
    }
    runningCommand.initialize();
  }

  @Override
  public void execute() {
    if (runningCommand != null) {
      runningCommand.execute();
    }
  }

  @Override
  public boolean isFinished() {
    return runningCommand != null && runningCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (runningCommand != null) {
      runningCommand.end(interrupted);
    }
  }
}
