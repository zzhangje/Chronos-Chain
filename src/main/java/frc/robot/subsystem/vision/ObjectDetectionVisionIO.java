package frc.robot.subsystem.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionVisionIO {
  @AutoLog
  class ObjectDetectionVisionIOInputs {
    public boolean connected;
    public double[] timestamp;
    public GamePieceObservation[][] gamePieceObservations;
  }

  record GamePieceObservation(int id, Rotation2d pitch, Rotation2d yaw) {}

  default void updateInputs(ObjectDetectionVisionIOInputs inputs) {}
}
