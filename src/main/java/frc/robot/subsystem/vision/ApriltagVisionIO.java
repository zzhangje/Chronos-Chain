package frc.robot.subsystem.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface ApriltagVisionIO {
  @AutoLog
  class ApriltagVisionIOInputs {
    public boolean connected;
    public boolean hasUpdate;
    public boolean hasTargets;

    public PoseObservation[] poseObservations;
    public int[] ids;

    public TxTyObservation[] txTyObservations;
  }

  record PoseObservation(
      double timestamp, Pose3d cameraInField, double ambiguity, int tagCount, double avgDistance) {}

  record TxTyObservation(double timestamp, int id, double txRad, double tyRad, double distance) {}

  default void updateInputs(ApriltagVisionIOInputs inputs) {
    inputs.connected = false;
    inputs.poseObservations = new PoseObservation[0];
    inputs.ids = new int[0];
    inputs.txTyObservations = new TxTyObservation[0];
  }
}
