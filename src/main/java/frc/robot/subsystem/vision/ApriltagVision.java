package frc.robot.subsystem.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.dashboard.Alert;
import frc.lib.interfaces.VirtualSubsystem;
import frc.lib.math.GeomUtil;
import frc.reefscape.Field;
import frc.robot.RobotState;
import java.util.*;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

@ExtensionMethod({GeomUtil.class})
public class ApriltagVision extends VirtualSubsystem {

  public record VisionObservation(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}

  public record SingleTagVisionObservation(
      double timestamp, Pose2d pose, int id, double distance) {}

  private static SimCameraProperties OV9281_1280_800() {
    var prop = new SimCameraProperties();
    prop.setCalibration(1280, 800, Rotation2d.fromDegrees(75.0));
    prop.setCalibError(0.37, 0.06);
    prop.setFPS(45.0);
    prop.setAvgLatencyMs(14);
    prop.setLatencyStdDevMs(5);
    return prop;
  }

  @RequiredArgsConstructor
  public enum CameraId {
    CASUAL_MID_LEFT("CasualMidLeft"),
    CASUAL_MID_RIGHT("CasualMidRight"),
    REEF_BACK_LEFT("ReefBackLeft"),
    REEF_BACK_RIGHT("ReefBackRight");

    private final String name;

    @Override
    public String toString() {
      return name;
    }
  }

  private final ApriltagVisionIO casualMidLeftIo;
  private final ApriltagVisionIO casualMidRightIo;
  private final ApriltagVisionIO reefBackLeftIo;
  private final ApriltagVisionIO reefBackRightIo;

  private final ApriltagVisionIOInputsAutoLogged casualMidLeftInputs =
      new ApriltagVisionIOInputsAutoLogged();
  private final ApriltagVisionIOInputsAutoLogged casualMidRightInputs =
      new ApriltagVisionIOInputsAutoLogged();
  private final ApriltagVisionIOInputsAutoLogged reefBackLeftInputs =
      new ApriltagVisionIOInputsAutoLogged();
  private final ApriltagVisionIOInputsAutoLogged reefBackRightInputs =
      new ApriltagVisionIOInputsAutoLogged();

  private final List<VisionObservation> allGoodVisionObservations = new ArrayList<>();
  private final HashMap<CameraId, Map<Integer, SingleTagVisionObservation>>
      allSingleTagVisionObservations = new HashMap<>();
  private final List<Pose3d> allGoodRobotInField = new ArrayList<>();
  private final List<Pose2d> allGoodRobotInField2d = new ArrayList<>();
  private final List<Pose3d> allBadRobotInField = new ArrayList<>();
  private final List<Pose3d> allUsedTagInField = new ArrayList<>();

  private final Alert casualMidLeftOfflineAlert =
      new Alert(CameraId.CASUAL_MID_LEFT.toString() + " offline!", Alert.AlertType.WARNING);
  private final Alert casualMidRightOfflineAlert =
      new Alert(CameraId.CASUAL_MID_RIGHT.toString() + " offline!", Alert.AlertType.WARNING);
  private final Alert reefBackLeftOfflineAlert =
      new Alert(CameraId.REEF_BACK_LEFT.toString() + " offline!", Alert.AlertType.WARNING);
  private final Alert reefBackRightOfflineAlert =
      new Alert(CameraId.REEF_BACK_RIGHT.toString() + " offline!", Alert.AlertType.WARNING);

  private VisionSystemSim visionSystemSim = null;

  private ApriltagVision(
      ApriltagVisionIO casualMidLeftIo,
      ApriltagVisionIO casualMidRightIo,
      ApriltagVisionIO reefBackLeftIo,
      ApriltagVisionIO reefBackRightIo) {
    this.casualMidLeftIo = casualMidLeftIo;
    this.casualMidRightIo = casualMidRightIo;
    this.reefBackLeftIo = reefBackLeftIo;
    this.reefBackRightIo = reefBackRightIo;
  }

  public ApriltagVision createReal() {
    return new ApriltagVision(
        new ApriltagVisionIOPhoton(CameraId.CASUAL_MID_LEFT.toString()),
        new ApriltagVisionIOPhoton(CameraId.CASUAL_MID_RIGHT.toString()),
        new ApriltagVisionIOPhoton(CameraId.REEF_BACK_LEFT.toString()),
        new ApriltagVisionIOPhoton(CameraId.REEF_BACK_RIGHT.toString()));
  }

  public ApriltagVision createSim(Supplier<Pose2d> poseSupplier) {
    return new ApriltagVision(
        new ApriltagVisionIOPhotonSim(
            CameraId.CASUAL_MID_LEFT.toString(),
            OV9281_1280_800(),
            ApriltagVisionConfig.CASUAL_MID_LEFT_IN_ROBOT.toTransform3d(),
            visionSystemSim,
            poseSupplier),
        new ApriltagVisionIOPhotonSim(
            CameraId.CASUAL_MID_RIGHT.toString(),
            OV9281_1280_800(),
            ApriltagVisionConfig.CASUAL_MID_RIGHT_IN_ROBOT.toTransform3d(),
            visionSystemSim,
            poseSupplier),
        new ApriltagVisionIOPhotonSim(
            CameraId.REEF_BACK_LEFT.toString(),
            OV9281_1280_800(),
            ApriltagVisionConfig.REEF_BACK_LEFT_IN_ROBOT.toTransform3d(),
            visionSystemSim,
            poseSupplier),
        new ApriltagVisionIOPhotonSim(
            CameraId.REEF_BACK_RIGHT.toString(),
            OV9281_1280_800(),
            ApriltagVisionConfig.REEF_BACK_RIGHT_IN_ROBOT.toTransform3d(),
            visionSystemSim,
            poseSupplier));
  }

  public ApriltagVision createIO() {
    return new ApriltagVision(
        new ApriltagVisionIO() {},
        new ApriltagVisionIO() {},
        new ApriltagVisionIO() {},
        new ApriltagVisionIO() {});
  }

  @Override
  public void periodic() {
    updateInputs();

    casualMidLeftOfflineAlert.set(!casualMidLeftInputs.connected);
    casualMidRightOfflineAlert.set(!casualMidRightInputs.connected);
    reefBackLeftOfflineAlert.set(!reefBackLeftInputs.connected);
    reefBackRightOfflineAlert.set(!reefBackRightInputs.connected);

    allGoodVisionObservations.clear();
    allSingleTagVisionObservations.clear();
    allGoodRobotInField.clear();
    allGoodRobotInField2d.clear();
    allBadRobotInField.clear();
    allUsedTagInField.clear();

    allSingleTagVisionObservations.put(CameraId.CASUAL_MID_LEFT, new HashMap<>());
    allSingleTagVisionObservations.put(CameraId.CASUAL_MID_RIGHT, new HashMap<>());
    allSingleTagVisionObservations.put(CameraId.REEF_BACK_LEFT, new HashMap<>());
    allSingleTagVisionObservations.put(CameraId.REEF_BACK_RIGHT, new HashMap<>());

    updateRobotInField(
        CameraId.CASUAL_MID_LEFT,
        casualMidLeftInputs,
        ApriltagVisionConfig.CASUAL_MID_LEFT_IN_ROBOT,
        true);
    updateRobotInField(
        CameraId.CASUAL_MID_RIGHT,
        casualMidRightInputs,
        ApriltagVisionConfig.CASUAL_MID_RIGHT_IN_ROBOT,
        true);
    updateRobotInField(
        CameraId.REEF_BACK_LEFT,
        reefBackLeftInputs,
        ApriltagVisionConfig.REEF_BACK_LEFT_IN_ROBOT,
        true);
    updateRobotInField(
        CameraId.REEF_BACK_RIGHT,
        reefBackRightInputs,
        ApriltagVisionConfig.REEF_BACK_RIGHT_IN_ROBOT,
        true);

    if (anyCameraHasUpdate()) {
      Logger.recordOutput(
          "ApriltagVision/AllBadRobotInField", allBadRobotInField.toArray(Pose3d[]::new));
      Logger.recordOutput(
          "ApriltagVision/AllGoodRobotInField", allGoodRobotInField.toArray(Pose3d[]::new));
      Logger.recordOutput(
          "ApriltagVision/AllGoodRobotInField2d", allGoodRobotInField2d.toArray(Pose2d[]::new));
      Logger.recordOutput(
          "ApriltagVision/AllUsedTagInField", allUsedTagInField.toArray(Pose3d[]::new));

      for (var singleTagPoses : allSingleTagVisionObservations.entrySet()) {
        // singleTagPoses.getValue().values().forEach(pose ->
        // Odometry.getInstance().addSingleTagObservation(singleTagPoses.getKey(), pose));
      }

      // allGoodVisionObservations.stream()
      //     .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
      //     .forEach(Odometry.getInstance()::addVisionObservation);
    }
  }

  private void updateInputs() {
    casualMidLeftIo.updateInputs(casualMidLeftInputs);
    casualMidRightIo.updateInputs(casualMidRightInputs);
    reefBackLeftIo.updateInputs(reefBackLeftInputs);
    reefBackRightIo.updateInputs(reefBackRightInputs);

    Logger.processInputs(
        "ApriltagVision/" + CameraId.CASUAL_MID_LEFT.toString(), casualMidLeftInputs);
    Logger.processInputs(
        "ApriltagVision/" + CameraId.CASUAL_MID_RIGHT.toString(), casualMidRightInputs);
    Logger.processInputs(
        "ApriltagVision/" + CameraId.REEF_BACK_LEFT.toString(), reefBackLeftInputs);
    Logger.processInputs(
        "ApriltagVision/" + CameraId.REEF_BACK_RIGHT.toString(), reefBackRightInputs);
  }

  private void updateRobotInField(
      CameraId cameraId,
      ApriltagVisionIOInputsAutoLogged inputs,
      Pose3d cameraInRobot,
      boolean isReefCamera) {
    if (!(inputs.connected
        && inputs.hasUpdate
        && inputs.hasTargets
        && inputs.poseObservations.length != 0
        && inputs.ids.length != 0
        && inputs.txTyObservations.length != 0)) {
      return;
    }

    for (final var poseObservation : inputs.poseObservations) {
      if (poseObservation.ambiguity() > ApriltagVisionConfig.MAX_ALLOWABLE_AMBIGUITY) {
        continue;
      }

      final var robotInField =
          poseObservation.cameraInField().transformBy(cameraInRobot.toTransform3d().inverse());

      if (robotInField.getX() < -ApriltagVisionConfig.FIELD_BORDER_THRESHOLD_METER
          || robotInField.getX() > Field.LENGTH + ApriltagVisionConfig.FIELD_BORDER_THRESHOLD_METER
          || robotInField.getY() < -ApriltagVisionConfig.FIELD_BORDER_THRESHOLD_METER
          || robotInField.getY() > Field.WIDTH + ApriltagVisionConfig.FIELD_BORDER_THRESHOLD_METER
          || robotInField.getZ() > ApriltagVisionConfig.ROBOT_POSE_Z_THRESHOLD_METER
          || robotInField.getZ() < -ApriltagVisionConfig.ROBOT_POSE_Z_THRESHOLD_METER) {
        allBadRobotInField.add(robotInField);
        continue;
      }

      final var xyStdDev =
          ApriltagVisionConfig.XY_STD_DEV_COEFF
              * Math.pow(poseObservation.avgDistance(), 2)
              / poseObservation.tagCount();

      final var thetaStdDev =
          poseObservation.tagCount() > 1
              ? ApriltagVisionConfig.THETA_STD_DEV_COEFF
                  * Math.pow(poseObservation.avgDistance(), 2)
                  / poseObservation.tagCount()
              : Double.POSITIVE_INFINITY;

      final var robotInField2d = robotInField.toPose2d();
      allGoodVisionObservations.add(
          new VisionObservation(
              poseObservation.timestamp(),
              robotInField2d,
              VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
      allGoodRobotInField.add(robotInField);
      allGoodRobotInField2d.add(robotInField2d);
    }

    for (final var id : inputs.ids) {
      allUsedTagInField.add(Field.APRILTAG_LAYOUT.getLayout().getTagPose(id).get());
    }

    if (!isReefCamera) {
      return;
    }

    final var odometry = RobotState.getOdometry();
    final var wheeledPose = odometry.getWheeledPose();
    final var estimatedPose = odometry.getEstimatedPose();

    for (final var observation : inputs.txTyObservations) {
      final var oldWheeledPose = odometry.getWheeledPoseByTimestamp(observation.timestamp());

      if (oldWheeledPose.isEmpty()) {
        continue;
      }
      final var robotRotation =
          estimatedPose
              .transformBy(new Transform2d(wheeledPose, oldWheeledPose.get()))
              .getRotation();

      final var camToTagTranslation =
          new Pose3d(
                  Translation3d.kZero, new Rotation3d(0, observation.tyRad(), -observation.txRad()))
              .transformBy(
                  new Transform3d(
                      new Translation3d(observation.distance(), 0, 0), Rotation3d.kZero))
              .getTranslation()
              .rotateBy(new Rotation3d(0, cameraInRobot.getRotation().getY(), 0))
              .toTranslation2d();

      final var camToTagRotation =
          robotRotation.plus(
              cameraInRobot.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));

      final var tagInField = Field.APRILTAG_LAYOUT.getLayout().getTagPose(observation.id()).get();
      final var cameraInFieldTranslation =
          new Pose2d(tagInField.toPose2d().getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
              .transformBy(GeomUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
              .getTranslation();
      var robotInField =
          new Pose2d(
                  cameraInFieldTranslation,
                  robotRotation.plus(cameraInRobot.toPose2d().getRotation()))
              .transformBy(new Transform2d(cameraInRobot.toPose2d(), Pose2d.kZero));
      robotInField = new Pose2d(robotInField.getTranslation(), robotRotation);

      var singleTagVisionObservations = allSingleTagVisionObservations.get(cameraId);
      if (!singleTagVisionObservations.containsKey(observation.id())
          || observation.timestamp()
              > singleTagVisionObservations.get(observation.id()).timestamp()) {
        singleTagVisionObservations.put(
            observation.id(),
            new SingleTagVisionObservation(
                observation.timestamp(), robotInField, observation.id(), observation.distance()));
      }
    }
  }

  private boolean singleCameraHasTagById(ApriltagVisionIOInputsAutoLogged inputs, int id) {
    if (!inputs.connected || inputs.ids == null) {
      return false;
    }
    return Arrays.stream(inputs.ids).anyMatch(seemedId -> seemedId == id);
  }

  public boolean reefCameraHasTagById(int id) {
    return singleCameraHasTagById(reefBackLeftInputs, id)
        || singleCameraHasTagById(reefBackRightInputs, id)
        || singleCameraHasTagById(casualMidLeftInputs, id)
        || singleCameraHasTagById(casualMidRightInputs, id);
  }

  private boolean anyCameraHasUpdate() {
    return reefBackLeftInputs.hasUpdate
        || reefBackRightInputs.hasUpdate
        || casualMidLeftInputs.hasUpdate
        || casualMidRightInputs.hasUpdate;
  }
}
