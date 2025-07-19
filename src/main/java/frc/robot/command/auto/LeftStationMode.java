package frc.robot.command.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.interfaces.ComposedCommands;
import frc.reefscape.Field;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotGoal;
import frc.robot.command.PounceCoralCommand;
import frc.robot.command.UniversalScoreCommand;
import frc.robot.command.general.WaitAllianceConfirmed;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.command.FollowTrajectory;

public class LeftStationMode extends ComposedCommands {
  public LeftStationMode(Swerve swerve, Arm arm, Intake intake) {
    setName("Auto/Left Station Mode");
    addRequirements(swerve, arm, intake);

    runningCommand =
        Commands.sequence(
            new WaitAllianceConfirmed(),
            RobotState.getOdometry().resetPoseCommand(() -> Field.LEFT_START_POSE),
            arm.idle(),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.scoreCoral("J", "4")),
            Commands.parallel(
                Commands.waitSeconds(0.4).andThen(arm.idle()),
                new FollowTrajectory(
                        swerve, () -> RobotState.getTrajectorySet().j2LeftCoralStation.get())
                    .withTimeout(1.5)),
            PounceCoralCommand.leftStation(swerve, intake, arm),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.scoreCoral("K", "4")),
            Commands.parallel(
                Commands.waitSeconds(0.4).andThen(arm.idle()),
                new FollowTrajectory(
                        swerve, () -> RobotState.getTrajectorySet().k2LeftCoralStation.get())
                    .withTimeout(1.2)),
            PounceCoralCommand.leftStation(swerve, intake, arm),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.scoreCoral("L", "4")),
            Commands.parallel(
                Commands.waitSeconds(0.4).andThen(arm.idle()),
                new FollowTrajectory(
                        swerve, () -> RobotState.getTrajectorySet().l2LeftCoralStation.get())
                    .withTimeout(1.1)),
            PounceCoralCommand.leftStation(swerve, intake, arm),
            new UniversalScoreCommand(swerve, arm, intake, () -> RobotGoal.scoreCoral("A", "4")));
  }
}
