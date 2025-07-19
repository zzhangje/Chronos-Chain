package frc.robot.command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.interfaces.ComposedCommands;
import frc.reefscape.Field.Barge;
import frc.robot.Constants.Misc;
import frc.robot.RobotContainer;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;
import frc.robot.subsystem.arm.ArmGoal.EndEffectorGoal;
import frc.robot.subsystem.arm.command.SetArmGoalCommand;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.command.ProceedToNet;
import java.util.function.DoubleSupplier;

public class MagicScoreNetCommand extends ComposedCommands {
  private final Swerve swerve;
  private final Arm arm;
  private final DoubleSupplier scoreDistanceSupplier;

  public MagicScoreNetCommand(Swerve swerve, Arm arm, DoubleSupplier scoreDistanceSupplier) {
    setName("Super/Magic Net Score");
    addRequirements(swerve, arm);

    this.swerve = swerve;
    this.arm = arm;
    this.scoreDistanceSupplier = scoreDistanceSupplier;
  }

  @Override
  public void initialize() {

    var currentPose = RobotContainer.getOdometry().getEstimatedPose();
    var goalPose =
        Barge.getAlgaeScoredPose(
            new Pose2d(
                currentPose.getX(),
                scoreDistanceSupplier.getAsDouble(),
                currentPose.getRotation()));
    System.out.println("MagicScoreNetCommand: goalPose = " + goalPose);
    var isLeft = Barge.closestRobotSide(RobotContainer.getOdometry().getEstimatedPose());
    var driveCmd =
        new ProceedToNet(swerve, () -> goalPose.plus(isLeft ? Misc.leftSide : Misc.rightSide));
    runningCommand =
        Commands.parallel(
                driveCmd,
                Commands.sequence(
                    Commands.waitUntil(
                        () -> driveCmd.hasHeadingAtGoal() && driveCmd.hasDistanceWithin(1.2)),
                    new SetArmGoalCommand(
                        arm,
                        () -> ArmSubsystemGoal.ALGAE_NET_SCORE.setIsLeft(() -> isLeft),
                        () -> true),
                    Commands.waitUntil(driveCmd::isFinished),
                    Commands.runOnce(() -> arm.setEeGoal(EndEffectorGoal.ALGAE_SCORE))))
            .withDeadline(
                Commands.waitUntil(() -> !arm.hasAlgae() || arm.hasCoral())
                    .finallyDo(() -> arm.setEeGoal(EndEffectorGoal.IDLE)));
    runningCommand.initialize();
  }
}
