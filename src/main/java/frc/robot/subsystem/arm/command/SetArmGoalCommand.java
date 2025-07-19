package frc.robot.subsystem.arm.command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.interfaces.ComposedCommands;
import frc.lib.math.EqualsUtil;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SetArmGoalCommand extends ComposedCommands {
  private final Arm arm;
  private final Supplier<ArmSubsystemGoal> goalSupplier;
  private final BooleanSupplier needStopSupplier;

  public SetArmGoalCommand(
      Arm arm, Supplier<ArmSubsystemGoal> goalSupplier, BooleanSupplier needStopSupplier) {
    this.arm = arm;
    this.goalSupplier = goalSupplier;
    this.needStopSupplier = needStopSupplier;
    addRequirements(arm);
    setName("Arm/Set Arm Goal");

    var goal = goalSupplier.get();
    var waitCommand =
        new WaitUntilCommand(
            () ->
                (needStopSupplier.getAsBoolean() && arm.stopAtGoal())
                    || (!needStopSupplier.getAsBoolean() && arm.atGoal()));
    if (EqualsUtil.epsilonEquals(
            goal.getShoulderHeightMeter(), ArmSubsystemGoal.HOME.getShoulderHeightMeter())
        && EqualsUtil.epsilonEquals(
            goal.getElbowPositionRad(), ArmSubsystemGoal.HOME.getElbowPositionRad())) {
      runningCommand = new HomeCommand(arm).andThen(new InstantCommand(() -> arm.setArmGoal(goal)));
    } else if (TransitionCommand.needsTransition(arm.getArmGoal(), goal)) {
      runningCommand =
          new TransitionCommand(arm, goalSupplier).andThen(new WaitUntilCommand(arm::stopAtGoal));
    } else {
      runningCommand = Commands.runOnce(() -> arm.setArmGoal(goal)).andThen(waitCommand);
    }
  }
}
