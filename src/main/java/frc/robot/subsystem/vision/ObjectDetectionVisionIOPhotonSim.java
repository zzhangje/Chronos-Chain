package frc.robot.subsystem.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.reefscape.GamePiece;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class ObjectDetectionVisionIOPhotonSim extends ObjectDetectionVisionIOPhoton {
  private static final TargetModel CORAL_MODEL = new TargetModel(GamePiece.Coral.LENGTH);
  private final VisionSystemSim visionSystemSim;
  private final Supplier<List<Pose3d>> unpickedCoralSupplier;
  private final Supplier<Pose2d> poseSupplier;

  public ObjectDetectionVisionIOPhotonSim(
      String cameraName,
      SimCameraProperties simCameraProperties,
      Transform3d robot2Camera,
      VisionSystemSim visionSystemSim,
      Supplier<Pose2d> poseSupplier,
      Supplier<List<Pose3d>> unpickedCoralSupplier) {
    super(cameraName);

    var sim = new PhotonCameraSim(super.camera, simCameraProperties);
    sim.enableProcessedStream(true);

    this.visionSystemSim = visionSystemSim;
    visionSystemSim.addCamera(sim, robot2Camera);
    this.unpickedCoralSupplier = unpickedCoralSupplier;
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void updateInputs(ObjectDetectionVisionIOInputs inputs) {
    visionSystemSim.update(poseSupplier.get());
    visionSystemSim.removeVisionTargets(GamePiece.GamePieceType.CORAL.getName());
    var unpickedCoral = unpickedCoralSupplier.get();
    var corals = new VisionTargetSim[unpickedCoral.size()];
    for (int i = 0; i < corals.length; i++) {
      var coral = unpickedCoral.get(i);
      var pose = new Pose3d(coral.getX(), coral.getY(), 0.0, new Rotation3d());
      corals[i] = new VisionTargetSim(pose, CORAL_MODEL);
    }
    visionSystemSim.addVisionTargets(GamePiece.GamePieceType.CORAL.getName(), corals);

    super.updateInputs(inputs);
  }
}
