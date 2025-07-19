package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.interfaces.ComposedCommands;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;
import frc.robot.subsystem.arm.ArmGoal.EndEffectorGoal;
import frc.robot.subsystem.arm.command.SetArmGoalCommand;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeGoal.IntakePivotGoal;
import frc.robot.subsystem.intake.IntakeGoal.IntakeRollerGoal;

public class RegraspCoralCommand extends ComposedCommands {
  public RegraspCoralCommand(Arm arm, Intake intake) {
    addRequirements(arm, intake);
    setName("Super/Regrasp Coral");

    runningCommand =
        Commands.sequence(
                Commands.parallel(
                    new SetArmGoalCommand(
                        arm, () -> ArmSubsystemGoal.CORAL_GROUND_PICK, () -> true),
                    Commands.runOnce(() -> intake.setPivotGoal(IntakePivotGoal.REGRASP))
                        .andThen(Commands.waitUntil(intake::isAtSetpoint))),
                Commands.runOnce(
                    () -> {
                      intake.setRollerGoal(IntakeRollerGoal.EJECT);
                      arm.setEeGoal(EndEffectorGoal.CORAL_COLLECT);
                    }))
            .withDeadline(Commands.waitUntil(() -> arm.hasCoral() || !intake.hasCoral()))
            .finallyDo(() -> intake.idle());
  }
}
