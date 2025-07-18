package frc.lib.dashboard;

import edu.wpi.first.wpilibj.DriverStation;

public record AllianceValue<T>(T blue, T red) {
  public T get() {
    var alliance = DriverStation.getAlliance();
    return get(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue);
  }

  public T get(DriverStation.Alliance alliance) {
    return get(alliance == DriverStation.Alliance.Blue);
  }

  public T get(boolean isBlueAlliance) {
    return isBlueAlliance ? blue : red;
  }
}
