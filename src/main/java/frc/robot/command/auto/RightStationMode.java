package frc.robot.command.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.interfaces.ComposedCommands;
import frc.reefscape.Field;
import frc.reefscape.RobotGoal;
import frc.robot.RobotContainer;
import frc.robot.command.PounceCoralCommand;
import frc.robot.command.UniversalScoreCommand;
import frc.robot.command.general.WaitAllianceConfirmed;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.command.FollowTrajectory;

public class RightStationMode extends ComposedCommands {
  public RightStationMode(Swerve swerve, Arm arm, Intake intake) {
    setName("Auto/Right Station Mode");
    addRequirements(swerve, arm, intake);

    runningCommand =
        Commands.sequence(
            new WaitAllianceConfirmed(),
            RobotContainer.getOdometry().resetPoseCommand(() -> Field.RIGHT_START_POSE),
            arm.idle(),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.scoreCoral("E", "4")),
            Commands.parallel(
                Commands.waitSeconds(0.4).andThen(arm.idle()),
                new FollowTrajectory(
                        swerve, () -> RobotContainer.getTrajectorySet().e2RightCoralStation.get())
                    .withTimeout(1.5)),
            PounceCoralCommand.rightStation(swerve, intake, arm),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.scoreCoral("D", "4")),
            Commands.parallel(
                Commands.waitSeconds(0.4).andThen(arm.idle()),
                new FollowTrajectory(
                        swerve, () -> RobotContainer.getTrajectorySet().d2RightCoralStation.get())
                    .withTimeout(1.2)),
            PounceCoralCommand.rightStation(swerve, intake, arm),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.scoreCoral("C", "4")),
            Commands.parallel(
                Commands.waitSeconds(0.4).andThen(arm.idle()),
                new FollowTrajectory(
                        swerve, () -> RobotContainer.getTrajectorySet().c2RightCoralStation.get())
                    .withTimeout(1.1)),
            PounceCoralCommand.rightStation(swerve, intake, arm),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.scoreCoral("B", "4")));
  }
}
