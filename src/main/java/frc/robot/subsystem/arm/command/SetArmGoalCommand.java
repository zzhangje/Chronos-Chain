package frc.robot.subsystem.arm.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;
import java.util.function.Supplier;

public class SetArmGoalCommand extends Command {
  private final Arm arm;
  private final Supplier<ArmSubsystemGoal> goalSupplier;
  private Command runningCommand;

  public SetArmGoalCommand(Arm arm, Supplier<ArmSubsystemGoal> goalSupplier) {
    this.arm = arm;
    this.goalSupplier = goalSupplier;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    var goal = goalSupplier.get();
    // if (EqualsUtil.epsilonEquals(goal.getShoulderHeightMeter(),
    // ArmSubsystemGoal.HOME.getShoulderHeightMeter())
    // && EqualsUtil.epsilonEquals(goal.getElbowPositionRad(),
    // ArmSubsystemGoal.HOME.getElbowPositionRad())) {
    // runningCommand = new HomeCommand(arm).andThen(new InstantCommand(() ->
    // arm.setArmGoal(goal)));
    // } else if (TransitionCommand.needsTransition(arm.getArmGoal(), goal)) {
    //   runningCommand = new TransitionCommand(arm, goalSupplier).andThen(new
    // WaitUntilCommand(arm::stopAtGoal));
    // } else {
    runningCommand =
        Commands.runOnce(() -> arm.setArmGoal(goal)).andThen(new WaitUntilCommand(arm::stopAtGoal));
    // }
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
