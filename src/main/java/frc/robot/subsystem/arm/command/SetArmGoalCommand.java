package frc.robot.subsystem.arm.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.math.EqualsUtil;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;

public class SetArmGoalCommand extends Command {
  private final Arm arm;
  private final ArmSubsystemGoal goal;
  private Command runningCommand;

  public SetArmGoalCommand(Arm arm, ArmSubsystemGoal goal) {
    this.arm = arm;
    this.goal = goal;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    if (EqualsUtil.epsilonEquals(goal.getShoulderHeightMeter(), 0.0)
        && EqualsUtil.epsilonEquals(goal.getElbowPositionRad(), Math.PI / 2.0)) {
      runningCommand = new HomeCommand(arm).andThen(new InstantCommand(() -> arm.setArmGoal(goal)));
    } else if (TransitionCommand.needsTransition(arm.getArmGoal(), goal)) {
      runningCommand = new TransitionCommand(arm, goal).andThen(new WaitUntilCommand(arm::atGoal));
    } else {
      runningCommand =
          new InstantCommand(() -> arm.setArmGoal(goal)).andThen(new WaitUntilCommand(arm::atGoal));
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
