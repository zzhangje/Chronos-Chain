package frc.robot.subsystem.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.reefscape.GamePiece;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;

public class ObjectDetectionVisionIOPhoton implements ObjectDetectionVisionIO {
  protected final PhotonCamera camera;

  public ObjectDetectionVisionIOPhoton(String cameraName) {
    camera = new PhotonCamera(cameraName);
  }

  @Override
  public void updateInputs(ObjectDetectionVisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    if (!inputs.connected) {
      inputs.timestamp = new double[0];
      inputs.gamePieceObservations = new ObjectDetectionVisionIO.GamePieceObservation[0][];
      return;
    }

    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      inputs.timestamp = new double[0];
      inputs.gamePieceObservations = new ObjectDetectionVisionIO.GamePieceObservation[0][];
      return;
    }

    var timestamp = new LinkedList<Double>();
    var observations = new LinkedList<List<ObjectDetectionVisionIO.GamePieceObservation>>();

    for (var result : results) {
      if (result.hasTargets()) {
        timestamp.add(result.getTimestampSeconds());

        observations.add(
            result.getTargets().stream()
                .filter(target -> target.objDetectId == GamePiece.Coral.OBJ_DETECT_ID)
                .map(
                    target ->
                        new ObjectDetectionVisionIO.GamePieceObservation(
                            target.objDetectId,
                            Rotation2d.fromDegrees(target.getPitch()).unaryMinus(),
                            Rotation2d.fromDegrees(target.getYaw()).unaryMinus()))
                .toList());
      }
    }

    if (timestamp.size() != observations.size()) {
      return;
    }

    inputs.timestamp = new double[timestamp.size()];
    inputs.gamePieceObservations =
        new ObjectDetectionVisionIO.GamePieceObservation[observations.size()][];

    for (int i = 0; i < observations.size(); i++) {
      inputs.timestamp[i] = timestamp.get(i);
      inputs.gamePieceObservations[i] =
          new ObjectDetectionVisionIO.GamePieceObservation[observations.get(i).size()];
      for (int j = 0; j < observations.get(i).size(); j++) {
        inputs.gamePieceObservations[i][j] = observations.get(i).get(j);
      }
    }
  }
}
