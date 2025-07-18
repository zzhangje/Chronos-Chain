package frc.robot.subsystem.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.reefscape.Field;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class ApriltagVisionIOPhotonSim extends ApriltagVisionIOPhoton {
  private final Supplier<Pose2d> poseSupplier;
  private final VisionSystemSim visionSystemSim;

  public ApriltagVisionIOPhotonSim(
      String cameraName,
      SimCameraProperties simCameraProperties,
      Transform3d robot2Camera,
      VisionSystemSim visionSystemSim,
      Supplier<Pose2d> poseSupplier) {
    super(cameraName);
    this.poseSupplier = poseSupplier;

    var sim =
        new PhotonCameraSim(super.camera, simCameraProperties, Field.APRILTAG_LAYOUT.getLayout());
    sim.enableProcessedStream(true);

    this.visionSystemSim = visionSystemSim;
    this.visionSystemSim.addCamera(sim, robot2Camera);
  }

  @Override
  public void updateInputs(ApriltagVisionIOInputs inputs) {
    visionSystemSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
