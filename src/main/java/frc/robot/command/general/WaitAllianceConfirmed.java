package frc.robot.command.general;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitAllianceConfirmed extends Command {
  private static final int MAX_ALLIANCE_CONFIRM_COUNT = 2;

  private DriverStation.Alliance lastAlliance = null;
  private int allianceConfirmCount = 0;

  @Override
  public void execute() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      allianceConfirmCount = 0;
      return;
    }

    if (lastAlliance != null) {
      if (lastAlliance == alliance.get()) {
        allianceConfirmCount++;
      } else {
        allianceConfirmCount = 0;
      }
    } else {
      allianceConfirmCount = 0;
    }

    lastAlliance = alliance.get();
  }

  @Override
  public boolean isFinished() {
    return allianceConfirmCount >= MAX_ALLIANCE_CONFIRM_COUNT;
  }
}
