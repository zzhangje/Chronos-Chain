package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.lib.utils.AllianceFlipUtil;
import frc.reefscape.Field;
import frc.robot.subsystem.swerve.Swerve.WheeledObservation;
import frc.robot.subsystem.swerve.SwerveConfig;
import frc.robot.subsystem.vision.ApriltagVision;
import frc.robot.subsystem.vision.ApriltagVision.SingleTagVisionObservation;
import frc.robot.subsystem.vision.ApriltagVision.VisionObservation;
import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Odometry {
  private static final LoggedTunableNumber txTyObservationStaleSecs =
      new LoggedTunableNumber(
          Constants.DebugGroup.ODOMETRY, "Odometry/TxTyObservation/StaleSeconds", 0.2);
  private static final LoggedTunableNumber txTyObservationMaxAllowedDistance =
      new LoggedTunableNumber(
          Constants.DebugGroup.ODOMETRY, "Odometry/TxTyObservation/MaxAllowedDistance", 2.0);

  private static final double POSE_BUFFER_SIZE_SEC = 2.0;
  private static final Matrix<N3, N1> WHEELED_STD_DEVS =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));

  @AutoLogOutput(key = "Odometry/WheeledPose")
  @Getter
  private Pose2d wheeledPose;

  @AutoLogOutput(key = "Odometry/EstimatedPose")
  @Getter
  private Pose2d estimatedPose;

  private final HashMap<ApriltagVision.CameraId, HashMap<Integer, SingleTagVisionObservation>>
      singleTagPoses = new HashMap<>();

  @Getter private Twist2d robotCentricVel = new Twist2d();
  private Twist2d trajectoryVel = new Twist2d();
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SEC);
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d lastGyroYaw = new Rotation2d();

  protected Odometry() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(WHEELED_STD_DEVS.get(i, 0), 2));
    }
    kinematics = SwerveConfig.SWERVE_KINEMATICS;

    resetPose(new Pose2d(Field.LENGTH / 2.0, Field.WIDTH / 2.0, Rotation2d.fromDegrees(0.0)));
  }

  public Command resetPoseCommand(Pose2d pose) {
    return Commands.runOnce(() -> resetPose(AllianceFlipUtil.apply(pose)));
  }

  public void addWheeledObservation(WheeledObservation observation) {
    var wheelPositions = observation.wheelPositions();
    var twist = kinematics.toTwist2d(lastWheelPositions, wheelPositions);
    lastWheelPositions = wheelPositions;
    if (observation.yaw() != null) {
      twist = new Twist2d(twist.dx, twist.dy, observation.yaw().minus(lastGyroYaw).getRadians());
      lastGyroYaw = observation.yaw();
    }
    wheeledPose = wheeledPose.exp(twist);
    poseBuffer.addSample(observation.timestamp(), wheeledPose);
    estimatedPose = estimatedPose.exp(twist);
  }

  public Optional<Pose2d> getWheeledPoseByTimestamp(double timestamp) {
    return poseBuffer.getSample(timestamp);
  }

  public void addVisionObservation(VisionObservation observation) {
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - POSE_BUFFER_SIZE_SEC
          > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      return;
    }

    var old2NowWheeledPoseTransform = new Transform2d(sample.get(), wheeledPose);
    var now2OldWheeledPoseTransform = new Transform2d(wheeledPose, sample.get());
    var oldEstimatedPose = estimatedPose.plus(now2OldWheeledPoseTransform);

    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }
    var visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }

    var transform = new Transform2d(oldEstimatedPose, observation.pose());
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    var scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));
    estimatedPose = oldEstimatedPose.plus(scaledTransform).plus(old2NowWheeledPoseTransform);
  }

  public void addSingleTagObservation(
      ApriltagVision.CameraId id, SingleTagVisionObservation observation) {
    var map = singleTagPoses.get(id);

    if (map.get(observation.id()).timestamp() >= observation.timestamp()) {
      return;
    }

    map.put(observation.id(), observation);
  }

  public Optional<Pose2d> getReefSingleTagPose(String branch, boolean isL1Scoring) {
    if (!Constants.ENABLE_SINGLE_TAG_POSE) {
      return Optional.of(getEstimatedPose());
    }

    ApriltagVision.CameraId cameraId;
    // FIXME: 删除单码模式
    if ("ACEGIK".contains(branch)) {
      cameraId =
          isL1Scoring
              ? ApriltagVision.CameraId.CASUAL_MID_RIGHT
              : ApriltagVision.CameraId.REEF_BACK_RIGHT;
    } else if ("BDFHJL".contains(branch)) {
      cameraId =
          isL1Scoring
              ? ApriltagVision.CameraId.CASUAL_MID_RIGHT
              : ApriltagVision.CameraId.REEF_BACK_LEFT;
    } else {
      return Optional.empty();
    }

    var tagId = getReefSingleTagPoseIdBySelection(branch);

    if (tagId == -1 || !singleTagPoses.get(cameraId).containsKey(tagId)) {
      DriverStation.reportError("No tag with id: " + tagId, true);
      return Optional.empty();
    }

    var singleTagPose = singleTagPoses.get(cameraId).get(tagId);

    if (Timer.getTimestamp() - singleTagPose.timestamp() >= txTyObservationStaleSecs.get()
        || singleTagPose.distance() >= txTyObservationMaxAllowedDistance.get()) {
      return Optional.empty();
    }
    var oldWheeledPose = poseBuffer.getSample(singleTagPose.timestamp());

    // Latency compensate
    return oldWheeledPose.map(
        pose -> singleTagPose.pose().plus(new Transform2d(pose, wheeledPose)));
  }

  public void addRobotCentricVel(Twist2d vel) {
    robotCentricVel = vel;
  }

  public void addTrajectoryVel(Twist2d vel) {
    trajectoryVel = vel;
  }

  @AutoLogOutput(key = "Odometry/FieldCentricVel")
  public Twist2d getFieldCentricVel() {
    var fieldCentricTranslationVel =
        new Translation2d(robotCentricVel.dx, robotCentricVel.dy)
            .rotateBy(estimatedPose.getRotation());

    return new Twist2d(
        fieldCentricTranslationVel.getX(),
        fieldCentricTranslationVel.getY(),
        robotCentricVel.dtheta);
  }

  @AutoLogOutput(key = "Odometry/VelNorm")
  public double getLinerVelNorm() {
    var vel = getFieldCentricVel();
    return Math.hypot(vel.dx, vel.dy);
  }

  public void resetPose(Pose2d initialPose) {
    estimatedPose = initialPose;
    wheeledPose = initialPose;
    poseBuffer.clear();

    singleTagPoses.clear();

    singleTagPoses.put(ApriltagVision.CameraId.CASUAL_MID_LEFT, new HashMap<>());
    singleTagPoses.put(ApriltagVision.CameraId.CASUAL_MID_RIGHT, new HashMap<>());
    singleTagPoses.put(ApriltagVision.CameraId.REEF_BACK_LEFT, new HashMap<>());
    singleTagPoses.put(ApriltagVision.CameraId.REEF_BACK_RIGHT, new HashMap<>());

    for (var entry : singleTagPoses.entrySet()) {
      var map = entry.getValue();
      for (int i = 1; i <= Field.APRILTAG_COUNT; i++) {
        map.put(
            i, new SingleTagVisionObservation(-999.0, new Pose2d(), i, Double.POSITIVE_INFINITY));
      }
    }
  }

  public Pose2d getPredictedPose(double translationLookaheadSec, double rotationLookaheadSec) {
    var vel = DriverStation.isAutonomousEnabled() ? trajectoryVel : robotCentricVel;
    return getEstimatedPose()
        .transformBy(
            new Transform2d(
                vel.dx * translationLookaheadSec,
                vel.dy * translationLookaheadSec,
                Rotation2d.fromRadians(vel.dtheta * rotationLookaheadSec)));
  }

  public static int getReefSingleTagPoseIdBySelection(String selectedBranch) {
    return switch (selectedBranch) {
      case "A", "B", "AB" -> AllianceFlipUtil.shouldFlip() ? 7 : 18;
      case "C", "D", "CD" -> AllianceFlipUtil.shouldFlip() ? 8 : 17;
      case "E", "F", "EF" -> AllianceFlipUtil.shouldFlip() ? 9 : 22;
      case "G", "H", "GH" -> AllianceFlipUtil.shouldFlip() ? 10 : 21;
      case "I", "J", "IJ" -> AllianceFlipUtil.shouldFlip() ? 11 : 20;
      case "K", "L", "KL" -> AllianceFlipUtil.shouldFlip() ? 6 : 19;
      default -> -1;
    };
  }

  public String getClosestCoralOrAlgaeNode(boolean isAlgae) {
    var robotPose = getEstimatedPose();
    var maybeFlippedPose = AllianceFlipUtil.apply(robotPose);

    return Field.Reef.L234_SCORE_POSES.entrySet().stream()
        .filter(
            entry -> entry.getKey().length() == (isAlgae ? 2 : 1) && !entry.getKey().equals("P"))
        .min(
            (entry1, entry2) -> {
              double dist1 =
                  maybeFlippedPose.getTranslation().getDistance(entry1.getValue().getTranslation());
              double dist2 =
                  maybeFlippedPose.getTranslation().getDistance(entry2.getValue().getTranslation());
              return Double.compare(dist1, dist2);
            })
        .map(Map.Entry::getKey)
        .orElse("");
  }
}
