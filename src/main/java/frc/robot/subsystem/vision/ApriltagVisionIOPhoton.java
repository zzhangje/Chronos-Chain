package frc.robot.subsystem.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.util.Units;
import frc.lib.math.GeomUtil;
import frc.reefscape.Field;
import java.util.ArrayList;
import java.util.HashSet;
import lombok.experimental.ExtensionMethod;
import org.photonvision.PhotonCamera;

@ExtensionMethod({GeomUtil.class})
public class ApriltagVisionIOPhoton implements ApriltagVisionIO {
  protected final PhotonCamera camera;
  private final AprilTagFieldLayout apriltagLayout = Field.APRILTAG_LAYOUT.getLayout();

  public ApriltagVisionIOPhoton(String cameraName) {
    PhotonCamera.setVersionCheckEnabled(false);

    camera = new PhotonCamera(cameraName);
  }

  @Override
  public void updateInputs(ApriltagVisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    inputs.hasUpdate = false;

    if (!inputs.connected) {
      inputs.hasTargets = false;
      inputs.poseObservations = new PoseObservation[0];
      inputs.ids = new int[0];
      inputs.txTyObservations = new TxTyObservation[0];
      return;
    }

    final var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return;
    }

    inputs.hasUpdate = true;
    inputs.hasTargets = false;

    final var ids = new HashSet<Short>();
    final var poseObservations = new ArrayList<PoseObservation>(results.size());
    final var txTyObservations = new ArrayList<TxTyObservation>(results.size() * 2);

    resultsLoop:
    for (final var result : results) {
      if (!result.hasTargets()) {
        continue;
      } else {
        inputs.hasTargets = true;
      }

      if (result.multitagResult.isPresent()) {
        final var multiTagResult = result.multitagResult.get();

        var sumDistance = 0.0;
        final var cameraInField = multiTagResult.estimatedPose.best.toPose3d();
        for (final var id : multiTagResult.fiducialIDsUsed) {
          final var tagPose = apriltagLayout.getTagPose((int) id);
          if (tagPose.isEmpty()) {
            continue resultsLoop;
          }
          sumDistance += cameraInField.getTranslation().getDistance(tagPose.get().getTranslation());
        }

        final var tagCount = multiTagResult.fiducialIDsUsed.size();
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                cameraInField,
                multiTagResult.estimatedPose.ambiguity,
                tagCount,
                sumDistance / tagCount));
        ids.addAll(multiTagResult.fiducialIDsUsed);
      } else {
        final var singleTagTarget = result.getBestTarget();

        final var apriltagIdUsed = singleTagTarget.getFiducialId();
        final var tagInField = apriltagLayout.getTagPose(apriltagIdUsed);
        final var tagInCamera = singleTagTarget.getBestCameraToTarget();
        final var id = singleTagTarget.getFiducialId();

        if (tagInField.isPresent()) {
          final var cameraInField = tagInField.get().plus(tagInCamera.inverse());
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(),
                  cameraInField,
                  singleTagTarget.getPoseAmbiguity(),
                  1,
                  cameraInField.getTranslation().getDistance(tagInField.get().getTranslation())));
          ids.add((short) id);
        }
      }

      for (final var target : result.getTargets()) {
        if (apriltagLayout.getTagPose(target.getFiducialId()).isEmpty()) {
          continue;
        }

        txTyObservations.add(
            new TxTyObservation(
                result.getTimestampSeconds(),
                target.getFiducialId(),
                Units.degreesToRadians(target.getYaw()),
                Units.degreesToRadians(target.getPitch()),
                target.getBestCameraToTarget().getTranslation().getNorm()));
      }
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    inputs.ids = new int[ids.size()];
    var i = 0;
    for (var id : ids) {
      inputs.ids[i++] = id;
    }

    inputs.txTyObservations = txTyObservations.toArray(new TxTyObservation[0]);
  }
}
