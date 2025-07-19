package frc.robot.subsystem.arm.command;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.lib.math.RotationUtil;
import frc.robot.Constants.DebugGroup;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;

public class TransitionCommand extends Command {
  private final Arm arm;
  private final ArmSubsystemGoal goal;
  private boolean needsAvoidReef;

  private static final LoggedTunableNumber transitionElevatorHeightMeter =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/TransitionElevatorHeightMeter", 0.85);
  private static final LoggedTunableNumber elbowAvoidReefAlgaePositionDegree =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/AvoidReefAlgaePositionDegree", 90.0);

  public TransitionCommand(Arm arm, ArmSubsystemGoal goal) {
    this.arm = arm;
    this.goal = goal;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    needsAvoidReef =
        !(goal.equals(ArmSubsystemGoal.ALGAE_HIGH_PICK)
            || goal.equals(ArmSubsystemGoal.ALGAE_LOW_PICK));
    arm.setShoulderPosition(transitionElevatorHeightMeter.get());
  }

  @Override
  public void execute() {
    if (needsAvoidReef
        && !arm.elbowAtPosition(Units.degreesToRadians(elbowAvoidReefAlgaePositionDegree.get()))) {
      arm.setElbowPosition(Units.degreesToRadians(elbowAvoidReefAlgaePositionDegree.get()));
    }
  }

  @Override
  public boolean isFinished() {
    if (!arm.shoulderAtPosition(transitionElevatorHeightMeter.get())) {
      return false;
    }

    if (needsShoulderFall(arm.getArmGoal(), goal)) {
      arm.setElbowPosition(goal.getElbowPositionRad());
      return arm.elbowAtPosition(goal.getElbowPositionRad());
    }

    return !needsAvoidReef
        || arm.elbowAtPosition(Units.degreesToRadians(elbowAvoidReefAlgaePositionDegree.get()));
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      arm.setArmGoal(goal);
    }
  }

  public static boolean needsTransition(ArmSubsystemGoal currentGoal, ArmSubsystemGoal targetGoal) {
    if (currentGoal == ArmSubsystemGoal.CORAL_GROUND_PICK
        || targetGoal == ArmSubsystemGoal.CORAL_GROUND_PICK) {
      return true;
    }

    if ((currentGoal == ArmSubsystemGoal.ALGAE_LOW_PICK
            || currentGoal == ArmSubsystemGoal.ALGAE_HIGH_PICK)
        || (targetGoal == ArmSubsystemGoal.ALGAE_LOW_PICK
            || targetGoal == ArmSubsystemGoal.ALGAE_HIGH_PICK)) {
      return !needsShoulderFall(currentGoal, targetGoal);
    }

    // FIXME: handle mirror case for elbow
    return RotationUtil.isRotationEnterSpecificAngleArea(
        currentGoal.getElbowPositionRad(),
        targetGoal.getElbowPositionRad(),
        Units.degreesToRadians(-180.0),
        Units.degreesToRadians(-60.0));
  }

  public static boolean needsShoulderFall(
      ArmSubsystemGoal currentGoal, ArmSubsystemGoal targetGoal) {
    double currentHeight = currentGoal.getShoulderHeightMeter();
    double targetHeight = targetGoal.getShoulderHeightMeter();
    double transitionHeight = transitionElevatorHeightMeter.get();

    return currentHeight > transitionHeight && targetHeight <= transitionHeight;
  }
}
