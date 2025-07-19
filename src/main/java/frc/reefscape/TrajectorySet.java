package frc.reefscape;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.lib.dashboard.AllianceValue;

public class TrajectorySet {
  public final AllianceValue<Trajectory<SwerveSample>> j2LeftCoralStation;
  public final AllianceValue<Trajectory<SwerveSample>> e2RightCoralStation;
  public final AllianceValue<Trajectory<SwerveSample>> k2LeftCoralStation;
  public final AllianceValue<Trajectory<SwerveSample>> l2LeftCoralStation;
  public final AllianceValue<Trajectory<SwerveSample>> c2RightCoralStation;
  public final AllianceValue<Trajectory<SwerveSample>> d2RightCoralStation;
  public final AllianceValue<Trajectory<SwerveSample>> ef2Net;
  public final AllianceValue<Trajectory<SwerveSample>> ij2Net;
  public final AllianceValue<Trajectory<SwerveSample>> gh2Net;

  public TrajectorySet() {
    j2LeftCoralStation = loadTrajectory("j2LeftCoralStation");
    e2RightCoralStation = loadTrajectory("e2RightCoralStation");
    k2LeftCoralStation = loadTrajectory("k2LeftCoralStation");
    l2LeftCoralStation = loadTrajectory("l2LeftCoralStation");
    c2RightCoralStation = loadTrajectory("c2RightCoralStation");
    d2RightCoralStation = loadTrajectory("d2RightCoralStation");
    ef2Net = loadTrajectory("ef2Net");
    ij2Net = loadTrajectory("ij2Net");
    gh2Net = loadTrajectory("gh2Net");
  }

  private AllianceValue<Trajectory<SwerveSample>> loadTrajectory(String name) {
    var trajectory = Choreo.loadTrajectory(name);
    if (trajectory.isEmpty()) {
      throw new NullPointerException("[TrajectoryLoader]: " + name + ".traj not found");
    }

    @SuppressWarnings("unchecked")
    var blue = (Trajectory<SwerveSample>) trajectory.get();
    System.out.println("[TrajectoryLoader]: " + name + ".traj loaded");

    return new AllianceValue<>(blue, blue.flipped());
  }
}
