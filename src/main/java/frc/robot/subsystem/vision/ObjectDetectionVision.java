package frc.robot.subsystem.vision;

import edu.wpi.first.math.geometry.*;
import frc.lib.dashboard.Alert;
import frc.lib.interfaces.VirtualSubsystem;
import frc.lib.math.GeomUtil;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

@ExtensionMethod({GeomUtil.class})
public class ObjectDetectionVision extends VirtualSubsystem {
  private static SimCameraProperties OV9782_1280_800() {
    var prop = new SimCameraProperties();
    prop.setCalibration(1280, 800, Rotation2d.fromDegrees(75.0));
    prop.setCalibError(0.37, 0.06);
    prop.setFPS(45.0);
    prop.setAvgLatencyMs(14);
    prop.setLatencyStdDevMs(5);
    return prop;
  }

  private final ObjectDetectionVisionIO frontMidIo;

  private final ObjectDetectionVisionIOInputsAutoLogged frontMidInputs =
      new ObjectDetectionVisionIOInputsAutoLogged();

  private final Alert frontMidCameraOfflineAlert =
      new Alert("Front mid camera offline!", Alert.AlertType.WARNING);

  public static ObjectDetectionVision createReal() {
    return new ObjectDetectionVision(new ObjectDetectionVisionIOPhoton("FrontMid"));
  }

  public static ObjectDetectionVision createSim(
      Supplier<Pose2d> poseSupplier, Supplier<Pose3d[]> unpickedCoralSupplier) {
    var visionSystemSim = new VisionSystemSim("object");
    return new ObjectDetectionVision(
        new ObjectDetectionVisionIOPhotonSim(
            "FrontMid",
            OV9782_1280_800(),
            ObjectDetectionVisionConfig.FRONT_MID_IN_ROBOT.toTransform3d(),
            visionSystemSim,
            poseSupplier,
            () -> Arrays.asList(unpickedCoralSupplier.get())));
  }

  public static ObjectDetectionVision createIO() {
    return new ObjectDetectionVision(new ObjectDetectionVisionIO() {});
  }

  private ObjectDetectionVision(ObjectDetectionVisionIO io) {
    frontMidIo = io;
  }

  @Override
  public void periodic() {
    frontMidIo.updateInputs(frontMidInputs);
    Logger.processInputs("ObjectDetectionVision/FrontMid", frontMidInputs);

    frontMidCameraOfflineAlert.set(!frontMidInputs.connected);

    updateGamePieces(frontMidInputs, ObjectDetectionVisionConfig.FRONT_MID_IN_ROBOT);
  }

  private void updateGamePieces(
      ObjectDetectionVisionIOInputsAutoLogged inputs, Pose3d cameraInRobot) {
    if (!inputs.connected) {
      return;
    }

    var resultsCount = inputs.timestamp.length;
    for (int i = 0; i < resultsCount; i++) {
      var robotInField = RobotContainer.getOdometry().getEstimatedPose();
      var allViewedCoralsInField = new ArrayList<Pose2d>();

      for (int j = 0; j < inputs.gamePieceObservations[i].length; j++) {
        var observation = inputs.gamePieceObservations[i][j];
        var robot2Coral =
            getRobot2GamePiece2d(cameraInRobot, observation.pitch(), observation.yaw(), 0.0);
        var coralInField = robotInField.transformBy(robot2Coral);
        allViewedCoralsInField.add(coralInField);
      }

      RobotContainer.getCoralTracker()
          .add(allViewedCoralsInField.toArray(Pose2d[]::new), inputs.timestamp[i]);
    }
  }

  private Transform2d getRobot2GamePiece2d(
      Pose3d cameraInRobot,
      Rotation2d targetPitchInCamera,
      Rotation2d targetYawInCamera,
      double targetHeightMeter) {
    var camera2TargetHeight = cameraInRobot.getZ() - targetHeightMeter;
    var camera2TargetPitchRad =
        targetPitchInCamera.getRadians() + cameraInRobot.getRotation().getY();
    var camera2TargetDistanceMeter = camera2TargetHeight / Math.tan(camera2TargetPitchRad);

    var camera2TargetYawRad = cameraInRobot.getRotation().getZ() + targetYawInCamera.getRadians();
    var camera2Target2d =
        new Translation2d(camera2TargetDistanceMeter, Rotation2d.fromRadians(camera2TargetYawRad));
    return cameraInRobot.toPose2d().toTransform2d().plus(camera2Target2d.toTransform2d());
  }
}
