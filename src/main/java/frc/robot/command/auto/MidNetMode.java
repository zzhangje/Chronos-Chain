package frc.robot.command.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.interfaces.ComposedCommands;
import frc.reefscape.Field;
import frc.reefscape.RobotGoal;
import frc.robot.RobotContainer;
import frc.robot.command.MagicScoreNetCommand;
import frc.robot.command.UniversalScoreCommand;
import frc.robot.command.general.WaitAllianceConfirmed;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.command.FollowTrajectory;

public class MidNetMode extends ComposedCommands {
  public MidNetMode(Swerve swerve, Arm arm, Intake intake) {
    setName("Auto/Left Station Mode");
    addRequirements(swerve, arm, intake);

    runningCommand =
        Commands.sequence(
            new WaitAllianceConfirmed(),
            RobotContainer.getOdometry().resetPoseCommand(() -> Field.MID_START_POSE),
            arm.idle(),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.scoreCoral("H", "4")),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.collectAlgae("GH")),
            Commands.parallel(
                Commands.waitSeconds(0.1).andThen(arm.algaeIdle()),
                new FollowTrajectory(swerve, () -> RobotContainer.getTrajectorySet().gh2Net.get())
                    .withTimeout(0.2)),
            new MagicScoreNetCommand(swerve, arm, () -> 4.65),
            arm.algaeIdle(),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.collectAlgae("IJ")),
            Commands.parallel(
                Commands.waitSeconds(0.1).andThen(arm.algaeIdle()),
                new FollowTrajectory(swerve, () -> RobotContainer.getTrajectorySet().ij2Net.get())
                    .withTimeout(0.2)),
            new MagicScoreNetCommand(swerve, arm, () -> 5.35),
            arm.algaeIdle(),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.collectAlgae("EF")),
            Commands.parallel(
                Commands.waitSeconds(0.1).andThen(arm.algaeIdle()),
                new FollowTrajectory(swerve, () -> RobotContainer.getTrajectorySet().ef2Net.get())
                    .withTimeout(0.2)),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.scoreProcessor()));
  }
}
