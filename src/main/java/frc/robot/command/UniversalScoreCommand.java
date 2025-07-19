package frc.robot.command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.interfaces.ComposedCommands;
import frc.lib.utils.AllianceFlipUtil;
import frc.reefscape.Field.Barge;
import frc.reefscape.Field.Processor;
import frc.reefscape.Field.Reef;
import frc.reefscape.GamePiece.GamePieceType;
import frc.robot.Constants.Misc;
import frc.robot.RobotContainer;
import frc.robot.RobotState.RobotGoal;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;
import frc.robot.subsystem.arm.ArmGoal.EndEffectorGoal;
import frc.robot.subsystem.arm.command.SetArmGoalCommand;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeGoal.IntakeRollerGoal;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.command.ProceedToNet;
import frc.robot.subsystem.swerve.command.ProceedToProcessor;
import frc.robot.subsystem.swerve.command.ProceedToReef;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class UniversalScoreCommand extends ComposedCommands {
  private final Swerve swerve;
  private final Arm arm;
  private final Intake intake;
  private final Supplier<RobotGoal> goalSupplier;

  @AutoLogOutput(key = "Super/IsLeft")
  private final BooleanSupplier isLeftSupplier;

  public UniversalScoreCommand(
      Swerve swerve,
      Arm arm,
      Intake intake,
      Supplier<RobotGoal> goalSupplier,
      BooleanSupplier isLeftSupplier) {
    this.swerve = swerve;
    this.arm = arm;
    this.intake = intake;
    this.goalSupplier = goalSupplier;
    this.isLeftSupplier = isLeftSupplier;
    addRequirements(swerve, arm, intake);
  }

  public UniversalScoreCommand(
      Swerve swerve, Arm arm, Intake intake, Supplier<RobotGoal> goalSupplier) {
    this(
        swerve,
        arm,
        intake,
        goalSupplier,
        () -> {
          if (goalSupplier.get().isScoreNet()) {
            return Barge.closestRobotSide(RobotContainer.getOdometry().getEstimatedPose());
          } else if (goalSupplier.get().isScoreProcessor()) {
            return Processor.closestRobotSide(RobotContainer.getOdometry().getEstimatedPose());
          } else {
            return Reef.closestRobotSide(RobotContainer.getOdometry().getEstimatedPose());
          }
        });
  }

  @Override
  public void initialize() {
    var goal = goalSupplier.get();
    System.out.println("UniversalScoreCommand initialized with goal:");
    System.out.println(
        String.format(
            "%s, %s, %s, %b",
            goal.getSelectedType().getName(),
            goal.getSelectedBranch(),
            goal.getSelectedLevel(),
            goal.getIgnoreArmMoveCondition()));
    if (goal.getSelectedType() == GamePieceType.ALGAE) {
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
                    Commands.either(
                        driveCmd, Commands.none(), () -> !goal.getIgnoreArmMoveCondition()),
                    Commands.sequence(
                        new SetArmGoalCommand(
                            arm,
                            () -> ArmSubsystemGoal.ALGAE_PROCESSOR_SCORE.setIsLeft(isLeftSupplier),
                            () -> true),
                        Commands.waitUntil(() -> arm.stopAtGoal()),
                        Commands.waitUntil(driveCmd::isFinished),
                        Commands.runOnce(() -> arm.setEeGoal(EndEffectorGoal.ALGAE_SCORE))))
                .withDeadline(
                    Commands.waitUntil(() -> !arm.hasAlgae() || arm.hasCoral())
                        .finallyDo(() -> arm.setEeGoal(EndEffectorGoal.IDLE)));
      } else if (goal.getSelectedBranch().equals("N")) {
        setName("Super/Score Net");
        var goalPose = Barge.getAlgaeScoredPose(RobotContainer.getOdometry().getEstimatedPose());
        var driveCmd =
            new ProceedToNet(
                swerve,
                () ->
                    goalPose.plus(isLeftSupplier.getAsBoolean() ? Misc.leftSide : Misc.rightSide));
        runningCommand =
            Commands.parallel(
                    Commands.either(
                        driveCmd, Commands.none(), () -> !goal.getIgnoreArmMoveCondition()),
                    Commands.sequence(
                        Commands.waitUntil(
                            () ->
                                goal.getIgnoreArmMoveCondition()
                                    || (driveCmd.hasHeadingAtGoal()
                                        && driveCmd.hasDistanceWithin(1.2))),
                        new SetArmGoalCommand(
                            arm,
                            () -> ArmSubsystemGoal.ALGAE_NET_SCORE.setIsLeft(isLeftSupplier),
                            () -> true),
                        Commands.waitUntil(driveCmd::isFinished),
                        Commands.runOnce(() -> arm.setEeGoal(EndEffectorGoal.ALGAE_SCORE))))
                .withDeadline(
                    Commands.waitUntil(() -> !arm.hasAlgae() || arm.hasCoral())
                        .finallyDo(() -> arm.setEeGoal(EndEffectorGoal.IDLE)));
      } else {
        setName("Super/Collect Algae");
        var goalPose =
            AllianceFlipUtil.apply(Reef.ALGAE_COLLECT_POSES.get(goal.getSelectedBranch()));
        var driveCmd =
            new ProceedToReef(
                swerve,
                () -> goalPose.plus(isLeftSupplier.getAsBoolean() ? Misc.leftSide : Misc.rightSide),
                () -> goalPose.getRotation(),
                () -> true);
        runningCommand =
            Commands.parallel(
                    Commands.either(
                        driveCmd, Commands.none(), () -> !goal.getIgnoreArmMoveCondition()),
                    Commands.sequence(
                        Commands.waitUntil(
                            () ->
                                goal.getIgnoreArmMoveCondition()
                                    || (driveCmd.hasHeadingAtGoal()
                                        && driveCmd.hasDistanceWithin(1.2))),
                        new SetArmGoalCommand(
                            arm,
                            () ->
                                (goal.isHighPick()
                                        ? ArmSubsystemGoal.ALGAE_HIGH_PICK
                                        : ArmSubsystemGoal.ALGAE_LOW_PICK)
                                    .setIsLeft(isLeftSupplier),
                            () -> true),
                        Commands.runOnce(() -> arm.setEeGoal(EndEffectorGoal.ALGAE_COLLECT))))
                .withDeadline(
                    Commands.waitUntil(() -> arm.hasAlgae() || arm.hasCoral())
                        .finallyDo(() -> arm.setEeGoal(EndEffectorGoal.IDLE)));
      }
    } else if (goal.getSelectedType() == GamePieceType.CORAL) {
      if (goal.getSelectedLevel().contains("1")) {
        setName("Super/Trough Score");
        var goalPose = AllianceFlipUtil.apply(Reef.L1_SCORE_POSES.get(goal.getSelectedBranch()));
        var driveCmd =
            new ProceedToReef(
                swerve,
                () -> goalPose,
                () -> goalPose.getRotation().rotateBy(Rotation2d.k180deg),
                () -> true);
        runningCommand =
            Commands.parallel(
                    Commands.either(
                        driveCmd, Commands.none(), () -> !goal.getIgnoreArmMoveCondition()),
                    Commands.waitUntil(
                        () ->
                            goal.getIgnoreArmMoveCondition()
                                || (driveCmd.hasHeadingAtGoal()
                                    && driveCmd.hasDistanceWithin(1.2))),
                    intake.idle(),
                    Commands.waitUntil(() -> intake.isAtSetpoint()),
                    Commands.waitUntil(driveCmd::isFinished),
                    Commands.runOnce(() -> intake.setRollerGoal(IntakeRollerGoal.TROUGH)))
                .withDeadline(
                    Commands.waitUntil(() -> !intake.hasCoral())
                        .finallyDo(() -> intake.setRollerGoal(IntakeRollerGoal.IDLE)));
      } else {
        setName("Super/Coral Score");
        var goalPose = AllianceFlipUtil.apply(Reef.L234_SCORE_POSES.get(goal.getSelectedBranch()));
        var driveCmd =
            new ProceedToReef(
                swerve,
                () -> goalPose.plus(isLeftSupplier.getAsBoolean() ? Misc.leftSide : Misc.rightSide),
                () -> goalPose.getRotation(),
                () -> true);
        runningCommand =
            Commands.parallel(
                    Commands.either(
                        driveCmd, Commands.none(), () -> !goal.getIgnoreArmMoveCondition()),
                    Commands.sequence(
                        new RegraspCoralCommand(arm, intake),
                        Commands.waitUntil(
                            () ->
                                goal.getIgnoreArmMoveCondition()
                                    || (driveCmd.hasHeadingAtGoal()
                                        && driveCmd.hasDistanceWithin(1.2))),
                        new SetArmGoalCommand(
                            arm,
                            () ->
                                (switch (goal.getSelectedLevel()) {
                                      case "2" -> ArmSubsystemGoal.CORAL_L2_PRESCORE;
                                      case "3" -> ArmSubsystemGoal.CORAL_L3_PRESCORE;
                                      case "4" -> ArmSubsystemGoal.CORAL_L4_PRESCORE;
                                      default -> ArmSubsystemGoal.IDLE;
                                    })
                                    .setIsLeft(isLeftSupplier),
                            () -> false),
                        Commands.waitUntil(driveCmd::isFinished),
                        Commands.parallel(
                            Commands.runOnce(() -> arm.setEeGoal(EndEffectorGoal.CORAL_SCORE)),
                            new SetArmGoalCommand(
                                arm,
                                () ->
                                    (switch (goal.getSelectedLevel()) {
                                          case "2" -> ArmSubsystemGoal.CORAL_L2_PRESCORE;
                                          case "3" -> ArmSubsystemGoal.CORAL_L3_PRESCORE;
                                          case "4" -> ArmSubsystemGoal.CORAL_L4_PRESCORE;
                                          default -> ArmSubsystemGoal.IDLE;
                                        })
                                        .setIsLeft(isLeftSupplier),
                                () -> false),
                            Commands.waitUntil(() -> !arm.hasCoral()))))
                .withDeadline(
                    Commands.waitUntil(
                        () -> (!intake.hasCoral() && !arm.hasCoral()) || arm.hasAlgae()))
                .finallyDo(() -> arm.setEeGoal(EndEffectorGoal.IDLE));
      }
    } else {
      setName("Super/Invalid Goal");
      runningCommand = Commands.none();
    }
    runningCommand.initialize();
  }
}
