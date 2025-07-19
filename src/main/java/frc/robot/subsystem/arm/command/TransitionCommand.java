package frc.robot.subsystem.arm.command;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.lib.math.RotationUtil;
import frc.robot.Constants.DebugGroup;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;
import java.util.function.Supplier;

public class TransitionCommand extends Command {
  private final Arm arm;
  private final Supplier<ArmSubsystemGoal> goalSupplier;
  private boolean needsAvoidReef;

  private static final LoggedTunableNumber transitionElevatorHeightMeter =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/TransitionElevatorHeightMeter", 0.85);
  private static final LoggedTunableNumber elbowAvoidReefAlgaePositionDegree =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/AvoidReefAlgaePositionDegree", 90.0);

  public TransitionCommand(Arm arm, Supplier<ArmSubsystemGoal> goalSupplier) {
    this.arm = arm;
    this.goalSupplier = goalSupplier;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    needsAvoidReef =
        !(goalSupplier.get().equals(ArmSubsystemGoal.ALGAE_HIGH_PICK)
            || goalSupplier.get().equals(ArmSubsystemGoal.ALGAE_LOW_PICK));
    // arm.setShoulderPosition(transitionElevatorHeightMeter.get());
  }

  @Override
  public void execute() {
    if (needsAvoidReef
        && !arm.elbowAtPosition(Units.degreesToRadians(elbowAvoidReefAlgaePositionDegree.get()))) {
      // arm.setElbowPosition(Units.degreesToRadians(elbowAvoidReefAlgaePositionDegree.get()));
    }
  }

  @Override
  public boolean isFinished() {
    if (!arm.shoulderAtPosition(transitionElevatorHeightMeter.get())) {
      return false;
    }

    if (needsShoulderFall(arm.getArmGoal(), goalSupplier.get())) {
      // arm.setElbowPosition(goalSupplier.get().getElbowPositionRad());
      return arm.elbowAtPosition(goalSupplier.get().getElbowPositionRad());
    }

    return !needsAvoidReef
        || arm.elbowAtPosition(Units.degreesToRadians(elbowAvoidReefAlgaePositionDegree.get()));
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      arm.setArmGoal(goalSupplier.get());
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

    if (targetGoal.getIsLeft()) {
      return RotationUtil.isRotationEnterSpecificAngleArea(
          currentGoal.getElbowPositionRad(),
          targetGoal.getElbowPositionRad(),
          Units.degreesToRadians(-180.0),
          Units.degreesToRadians(-60.0));
    } else {
      return RotationUtil.isRotationEnterSpecificAngleArea(
          currentGoal.getElbowPositionRad(),
          targetGoal.getElbowPositionRad(),
          Units.degreesToRadians(-120.0),
          Units.degreesToRadians(0.0));
    }
  }

  public static boolean needsShoulderFall(
      ArmSubsystemGoal currentGoal, ArmSubsystemGoal targetGoal) {
    double currentHeight = currentGoal.getShoulderHeightMeter();
    double targetHeight = targetGoal.getShoulderHeightMeter();
    double transitionHeight = transitionElevatorHeightMeter.get();

    return currentHeight > transitionHeight && targetHeight <= transitionHeight;
  }
}
