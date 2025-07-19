package frc.robot.subsystem.arm.command;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.Constants.DebugGroup;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;

public class HomeCommand extends Command {
  private final Arm arm;
  private final Timer timer = new Timer();
  private final Debouncer debouncer;
  private boolean hasElbowReachedGoal = false;

  protected static final LoggedTunableNumber shoulderHomingCurrentAmp =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/HomingCurrentAmp", -3.0);
  protected static final LoggedTunableNumber shoulderHomingTimeSecs =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/HomingTimeSecs", 0.2);
  protected static final LoggedTunableNumber shoulderHomingVelocityThreshMeterPerSec =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/HomingVelocityThreshMeterPerSec", 0.05);

  public HomeCommand(Arm arm) {
    this.arm = arm;
    this.debouncer = new Debouncer(shoulderHomingTimeSecs.get());
    addRequirements(arm);
    withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  @Override
  public void initialize() {
    arm.setArmGoal(ArmSubsystemGoal.HOME);
    timer.start();
    hasElbowReachedGoal = false;
    debouncer.calculate(false);
  }

  @Override
  public void execute() {
    if (!hasElbowReachedGoal && arm.elbowAtGoal()) {
      hasElbowReachedGoal = true;
    }

    if (hasElbowReachedGoal) {
      arm.setShoulderCurrent(shoulderHomingCurrentAmp.get());
    }
  }

  @Override
  public boolean isFinished() {
    return hasElbowReachedGoal
        && (arm.atGoal()
            || debouncer.calculate(
                Math.abs(arm.getShoulderVelMeterPerSec())
                    <= shoulderHomingVelocityThreshMeterPerSec.get()));
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      arm.homeShoulder(0.0);
    }
    arm.setArmGoal(ArmSubsystemGoal.IDLE);
    timer.stop();
  }
}
