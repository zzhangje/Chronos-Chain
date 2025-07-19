package frc.reefscape;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.utils.AllianceFlipUtil;
import frc.reefscape.GamePiece.Algae;
import frc.reefscape.GamePiece.Coral;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/** Class to store all field constants. Length/Height -> meter, Angle -> degree */
public final class Field {
  public static final double LENGTH = 17.548;
  public static final double WIDTH = 8.052;

  public static final Pose2d LEFT_START_POSE =
      new Pose2d(7.13088474273682, 5.654391765594482, Rotation2d.kZero);
  public static final Pose2d MID_START_POSE =
      new Pose2d(7.13088474273682, 4.016828536987305, Rotation2d.kCCW_90deg);
  public static final Pose2d RIGHT_START_POSE =
      new Pose2d(7.13088474273682, (8.052 - 5.654391765594482), Rotation2d.kZero);

  public static final Pose2d LEFT_ESCAPE_START_POSE =
      new Pose2d(7.13088474273682, 7.74282169342041, Rotation2d.kZero);

  public static Translation2d[] RIGHT_GROUND_PICK_SEARCH_AREA =
      new Translation2d[] {
        new Translation2d(0.0, 1.2323505878448486),
        new Translation2d(1.6639333963394165, 0.0),
        new Translation2d(4.491930961608887, 0.0),
        new Translation2d(4.491930961608887, 1.7343279123306274)
      };

  public static Translation2d[] LEFT_GROUND_PICK_SEARCH_AREA =
      new Translation2d[] {
        new Translation2d(0.0, WIDTH - 1.2323505878448486),
        new Translation2d(1.6639333963394165, WIDTH),
        new Translation2d(4.491930961608887, WIDTH),
        new Translation2d(4.491930961608887, WIDTH - 1.7343279123306274)
      };

  public static Translation2d[] MID_GROUND_PICK_SEARCH_AREA =
      new Translation2d[] {
        new Translation2d(2.7797515392303467, 5.867116451263428),
        new Translation2d(0.0, 5.867116451263428),
        new Translation2d(0, WIDTH - 5.867116451263428),
        new Translation2d(2.7797515392303467, WIDTH - 5.867116451263428)
      };

  public static Translation2d[] SELF_HALF_FIELD_SEARCH_AREA =
      new Translation2d[] {
        new Translation2d(0.0, Field.WIDTH),
        new Translation2d(Field.LENGTH / 2.0, Field.WIDTH),
        new Translation2d(Field.LENGTH / 2.0, 0.0),
        new Translation2d(0.0, 0.0)
      };

  public static class Barge {
    public static final double NET_HEIGHT = 2.27;
    public static final double BARGE_SCORE_X = 7.85;
    public static final Rotation2d BARGE_SCORE_HEADING = Rotation2d.k180deg;

    public static final Rotation2d CLIMBING_HEADING = Rotation2d.k180deg;
    public static final double CAGE_LEFT_Y = 7.267968654632568;
    public static final double CAGE_MID_Y = 6.1707329750061035;
    public static final double CAGE_RIGHT_Y = 5.062817573547363;

    public static final Pose3d[] SCORABLE_POSES;

    public static Boolean closestRobotSide(Pose2d pose) {
      return (!AllianceFlipUtil.inBlueHalf(pose) ? AllianceFlipUtil.mirror(pose) : pose)
              .getRotation()
              .getSin()
          < 0;
    }

    static {
      double LIMIT = 0.375;
      double DIAMETER = Algae.DIAMETER;
      SCORABLE_POSES =
          new Pose3d[] {
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 0 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 1 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 2 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 3 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 4 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 6 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 7 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 8 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 0.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 1.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 2.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 3.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 4.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 5.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 6.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 7.5 * DIAMETER, false),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 0.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 1.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 2.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 3.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 4.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 5.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 6.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 + LIMIT + 7.5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 0 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 1 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 2 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 3 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 4 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 5 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 6 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 7 * DIAMETER, true),
            Field.Barge.getPoseBasedOnY(Field.WIDTH / 2 - LIMIT - 8 * DIAMETER, true)
          };
    }

    // Get position based on Y coordinate
    private static Pose3d getPoseBasedOnY(double y, boolean closedToBlue) {
      return new Pose3d(
          Field.LENGTH / 2.0 + (closedToBlue ? 0.15 : -0.15),
          y,
          NET_HEIGHT - GamePiece.Algae.DIAMETER / 2.0,
          new Rotation3d());
    }

    public static Pose2d getAlgaeScoredPose(Pose2d pose) {
      if (AllianceFlipUtil.shouldFlip()) {
        if (AllianceFlipUtil.inBlueHalf(pose)) {
          return new Pose2d(
              BARGE_SCORE_X, Math.min(pose.getY(), Field.WIDTH / 2 - 0.4), BARGE_SCORE_HEADING);
        } else {
          return new Pose2d(
              AllianceFlipUtil.mirrorX(BARGE_SCORE_X),
              Math.min(pose.getY(), Field.WIDTH / 2 - 0.4),
              BARGE_SCORE_HEADING.plus(Rotation2d.k180deg));
        }
      } else {
        if (AllianceFlipUtil.inBlueHalf(pose)) {
          return new Pose2d(
              BARGE_SCORE_X, Math.max(pose.getY(), Field.WIDTH / 2 + 0.4), BARGE_SCORE_HEADING);
        } else {
          return new Pose2d(
              AllianceFlipUtil.mirrorX(BARGE_SCORE_X),
              Math.max(pose.getY(), Field.WIDTH / 2 + 0.4),
              BARGE_SCORE_HEADING.plus(Rotation2d.k180deg));
        }
      }
    }
  }

  public static class Reef {
    public static final Translation2d CENTER =
        new Translation2d(Units.inchesToMeters(176.746), WIDTH / 2.0);

    public static int getSingleTagPoseIdBySelection(String selectedBranch) {
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

    public static Boolean closestRobotSide(Pose2d pose) {
      Pose2d tmpPose = !AllianceFlipUtil.inBlueHalf(pose) ? AllianceFlipUtil.mirror(pose) : pose;
      Rotation2d theta = (tmpPose.getTranslation().minus(Reef.CENTER)).getAngle();
      return AllianceFlipUtil.inBlueHalf(pose)
          ? pose.getRotation().minus(theta).getSin() > 0.0
          : pose.getRotation().minus(theta).getSin() < 0.0;
    }

    public static final double FACE_LENGTH = Units.inchesToMeters(36.792600);
    public static final double CENTER_2_SIDE_DISTANCE = 1.178389072;

    public static final Map<String, Pose2d> L234_SCORE_POSES = new HashMap<>();
    public static final Map<String, Pose2d> L1_SCORE_POSES = new HashMap<>();
    public static final Map<String, Pose2d> ALGAE_COLLECT_POSES = new HashMap<>();

    // TODO: adjust this based on the field
    public static final double SCORE_ADJUST_X = 0.36 + GamePiece.Coral.DIAMETER * 1.0;
    // Caused by coral end effector is not in robot center line
    public static final double CORAL_ADJUST_Y_OFFSET = 0.0;
    // Caused by algae end effector is not in robot center line
    public static final double ALGAE_ADJUST_Y_OFFSET = 0.0;
    public static final double SCORE_ADJUST_Y = Units.inchesToMeters(6.519);
    public static final Rotation2d L1_SCORE_ADJUST_HEADING = Rotation2d.k180deg;

    static {
      L1_SCORE_POSES.put("A", getScorePoseBasedOnTag(18, -SCORE_ADJUST_Y, L1_SCORE_ADJUST_HEADING));
      L1_SCORE_POSES.put("B", getScorePoseBasedOnTag(18, SCORE_ADJUST_Y, L1_SCORE_ADJUST_HEADING));
      L1_SCORE_POSES.put("C", getScorePoseBasedOnTag(17, -SCORE_ADJUST_Y, L1_SCORE_ADJUST_HEADING));
      L1_SCORE_POSES.put("D", getScorePoseBasedOnTag(17, SCORE_ADJUST_Y, L1_SCORE_ADJUST_HEADING));
      L1_SCORE_POSES.put("E", getScorePoseBasedOnTag(22, -SCORE_ADJUST_Y, L1_SCORE_ADJUST_HEADING));
      L1_SCORE_POSES.put("F", getScorePoseBasedOnTag(22, SCORE_ADJUST_Y, L1_SCORE_ADJUST_HEADING));
      L1_SCORE_POSES.put("G", getScorePoseBasedOnTag(21, -SCORE_ADJUST_Y, L1_SCORE_ADJUST_HEADING));
      L1_SCORE_POSES.put("H", getScorePoseBasedOnTag(21, SCORE_ADJUST_Y, L1_SCORE_ADJUST_HEADING));
      L1_SCORE_POSES.put("I", getScorePoseBasedOnTag(20, -SCORE_ADJUST_Y, L1_SCORE_ADJUST_HEADING));
      L1_SCORE_POSES.put("J", getScorePoseBasedOnTag(20, SCORE_ADJUST_Y, L1_SCORE_ADJUST_HEADING));
      L1_SCORE_POSES.put("K", getScorePoseBasedOnTag(19, -SCORE_ADJUST_Y, L1_SCORE_ADJUST_HEADING));
      L1_SCORE_POSES.put("L", getScorePoseBasedOnTag(19, SCORE_ADJUST_Y, L1_SCORE_ADJUST_HEADING));

      L234_SCORE_POSES.put(
          "A", getScorePoseBasedOnTag(18, -SCORE_ADJUST_Y - CORAL_ADJUST_Y_OFFSET));
      L234_SCORE_POSES.put("B", getScorePoseBasedOnTag(18, SCORE_ADJUST_Y - CORAL_ADJUST_Y_OFFSET));
      L234_SCORE_POSES.put(
          "C", getScorePoseBasedOnTag(17, -SCORE_ADJUST_Y - CORAL_ADJUST_Y_OFFSET));
      L234_SCORE_POSES.put("D", getScorePoseBasedOnTag(17, SCORE_ADJUST_Y - CORAL_ADJUST_Y_OFFSET));
      L234_SCORE_POSES.put(
          "E", getScorePoseBasedOnTag(22, -SCORE_ADJUST_Y - CORAL_ADJUST_Y_OFFSET));
      L234_SCORE_POSES.put("F", getScorePoseBasedOnTag(22, SCORE_ADJUST_Y - CORAL_ADJUST_Y_OFFSET));
      L234_SCORE_POSES.put(
          "G", getScorePoseBasedOnTag(21, -SCORE_ADJUST_Y - CORAL_ADJUST_Y_OFFSET));
      L234_SCORE_POSES.put("H", getScorePoseBasedOnTag(21, SCORE_ADJUST_Y - CORAL_ADJUST_Y_OFFSET));
      L234_SCORE_POSES.put(
          "I", getScorePoseBasedOnTag(20, -SCORE_ADJUST_Y - CORAL_ADJUST_Y_OFFSET));
      L234_SCORE_POSES.put("J", getScorePoseBasedOnTag(20, SCORE_ADJUST_Y - CORAL_ADJUST_Y_OFFSET));
      L234_SCORE_POSES.put(
          "K", getScorePoseBasedOnTag(19, -SCORE_ADJUST_Y - CORAL_ADJUST_Y_OFFSET));
      L234_SCORE_POSES.put("L", getScorePoseBasedOnTag(19, SCORE_ADJUST_Y - CORAL_ADJUST_Y_OFFSET));

      ALGAE_COLLECT_POSES.put("AB", getScorePoseBasedOnTag(18, ALGAE_ADJUST_Y_OFFSET));
      ALGAE_COLLECT_POSES.put("CD", getScorePoseBasedOnTag(17, ALGAE_ADJUST_Y_OFFSET));
      ALGAE_COLLECT_POSES.put("EF", getScorePoseBasedOnTag(22, ALGAE_ADJUST_Y_OFFSET));
      ALGAE_COLLECT_POSES.put("GH", getScorePoseBasedOnTag(21, ALGAE_ADJUST_Y_OFFSET));
      ALGAE_COLLECT_POSES.put("IJ", getScorePoseBasedOnTag(20, ALGAE_ADJUST_Y_OFFSET));
      ALGAE_COLLECT_POSES.put("KL", getScorePoseBasedOnTag(19, ALGAE_ADJUST_Y_OFFSET));
    }

    public static final Map<String, Pose3d> CORAL_POSES = new HashMap<>();
    public static final double CORAL_ADJUST_X = -GamePiece.Coral.DIAMETER / 2.0;
    public static final double CORAL_ADJUST_Y = Units.inchesToMeters(6.469);
    public static final double TROUGH_1_DIAG = -0.18;
    public static final double TROUGH_2_DIAG = -0.08;

    static {
      CORAL_POSES.put(
          "A11", getTroughCoralPoseBasedOnTag(18, true, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "A12", getTroughCoralPoseBasedOnTag(18, true, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "B11",
          getTroughCoralPoseBasedOnTag(18, false, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "B12",
          getTroughCoralPoseBasedOnTag(18, false, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "C11", getTroughCoralPoseBasedOnTag(17, true, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "C12", getTroughCoralPoseBasedOnTag(17, true, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "D11",
          getTroughCoralPoseBasedOnTag(17, false, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "D12",
          getTroughCoralPoseBasedOnTag(17, false, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "E11", getTroughCoralPoseBasedOnTag(22, true, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "E12", getTroughCoralPoseBasedOnTag(22, true, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "F11",
          getTroughCoralPoseBasedOnTag(22, false, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "F12",
          getTroughCoralPoseBasedOnTag(22, false, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "G11", getTroughCoralPoseBasedOnTag(21, true, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "G12", getTroughCoralPoseBasedOnTag(21, true, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "H11",
          getTroughCoralPoseBasedOnTag(21, false, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "H12",
          getTroughCoralPoseBasedOnTag(21, false, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "I11", getTroughCoralPoseBasedOnTag(20, true, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "I12", getTroughCoralPoseBasedOnTag(20, true, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "J11",
          getTroughCoralPoseBasedOnTag(20, false, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "J12",
          getTroughCoralPoseBasedOnTag(20, false, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "K11", getTroughCoralPoseBasedOnTag(19, true, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "K12", getTroughCoralPoseBasedOnTag(19, true, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "L11",
          getTroughCoralPoseBasedOnTag(19, false, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "L12",
          getTroughCoralPoseBasedOnTag(19, false, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));

      CORAL_POSES.put(
          "#A11", getTroughCoralPoseBasedOnTag(7, true, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "#A12", getTroughCoralPoseBasedOnTag(7, true, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "#B11",
          getTroughCoralPoseBasedOnTag(7, false, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "#B12",
          getTroughCoralPoseBasedOnTag(7, false, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "#C11", getTroughCoralPoseBasedOnTag(8, true, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "#C12", getTroughCoralPoseBasedOnTag(8, true, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "#D11",
          getTroughCoralPoseBasedOnTag(8, false, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "#D12",
          getTroughCoralPoseBasedOnTag(8, false, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "#E11", getTroughCoralPoseBasedOnTag(9, true, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "#E12", getTroughCoralPoseBasedOnTag(9, true, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "#F11",
          getTroughCoralPoseBasedOnTag(9, false, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "#F12",
          getTroughCoralPoseBasedOnTag(9, false, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "#G11",
          getTroughCoralPoseBasedOnTag(10, true, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "#G12",
          getTroughCoralPoseBasedOnTag(10, true, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "#H11",
          getTroughCoralPoseBasedOnTag(10, false, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "#H12",
          getTroughCoralPoseBasedOnTag(10, false, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "#I11",
          getTroughCoralPoseBasedOnTag(11, true, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "#I12",
          getTroughCoralPoseBasedOnTag(11, true, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "#J11",
          getTroughCoralPoseBasedOnTag(11, false, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "#J12",
          getTroughCoralPoseBasedOnTag(11, false, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "#K11", getTroughCoralPoseBasedOnTag(6, true, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "#K12", getTroughCoralPoseBasedOnTag(6, true, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));
      CORAL_POSES.put(
          "#L11",
          getTroughCoralPoseBasedOnTag(6, false, ReefHeight.L11.heightMeter, TROUGH_1_DIAG));
      CORAL_POSES.put(
          "#L12",
          getTroughCoralPoseBasedOnTag(6, false, ReefHeight.L12.heightMeter, TROUGH_2_DIAG));

      CORAL_POSES.put(
          "A2",
          getCoralPoseBasedOnTag(18, true, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "A3",
          getCoralPoseBasedOnTag(18, true, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "A4",
          getCoralPoseBasedOnTag(18, true, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "B2",
          getCoralPoseBasedOnTag(18, false, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "B3",
          getCoralPoseBasedOnTag(18, false, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "B4",
          getCoralPoseBasedOnTag(18, false, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "C2",
          getCoralPoseBasedOnTag(17, true, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "C3",
          getCoralPoseBasedOnTag(17, true, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "C4",
          getCoralPoseBasedOnTag(17, true, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "D2",
          getCoralPoseBasedOnTag(17, false, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "D3",
          getCoralPoseBasedOnTag(17, false, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "D4",
          getCoralPoseBasedOnTag(17, false, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "E2",
          getCoralPoseBasedOnTag(22, true, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "E3",
          getCoralPoseBasedOnTag(22, true, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "E4",
          getCoralPoseBasedOnTag(22, true, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "F2",
          getCoralPoseBasedOnTag(22, false, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "F3",
          getCoralPoseBasedOnTag(22, false, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "F4",
          getCoralPoseBasedOnTag(22, false, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "G2",
          getCoralPoseBasedOnTag(21, true, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "G3",
          getCoralPoseBasedOnTag(21, true, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "G4",
          getCoralPoseBasedOnTag(21, true, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "H2",
          getCoralPoseBasedOnTag(21, false, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "H3",
          getCoralPoseBasedOnTag(21, false, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "H4",
          getCoralPoseBasedOnTag(21, false, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "I2",
          getCoralPoseBasedOnTag(20, true, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "I3",
          getCoralPoseBasedOnTag(20, true, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "I4",
          getCoralPoseBasedOnTag(20, true, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "J2",
          getCoralPoseBasedOnTag(20, false, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "J3",
          getCoralPoseBasedOnTag(20, false, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "J4",
          getCoralPoseBasedOnTag(20, false, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "K2",
          getCoralPoseBasedOnTag(19, true, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "K3",
          getCoralPoseBasedOnTag(19, true, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "K4",
          getCoralPoseBasedOnTag(19, true, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "L2",
          getCoralPoseBasedOnTag(19, false, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "L3",
          getCoralPoseBasedOnTag(19, false, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "L4",
          getCoralPoseBasedOnTag(19, false, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "#A2",
          getCoralPoseBasedOnTag(7, true, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "#A3",
          getCoralPoseBasedOnTag(7, true, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "#A4",
          getCoralPoseBasedOnTag(7, true, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "#B2",
          getCoralPoseBasedOnTag(7, false, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "#B3",
          getCoralPoseBasedOnTag(7, false, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "#B4",
          getCoralPoseBasedOnTag(7, false, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "#C2",
          getCoralPoseBasedOnTag(8, true, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "#C3",
          getCoralPoseBasedOnTag(8, true, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "#C4",
          getCoralPoseBasedOnTag(8, true, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "#D2",
          getCoralPoseBasedOnTag(8, false, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "#D3",
          getCoralPoseBasedOnTag(8, false, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "#D4",
          getCoralPoseBasedOnTag(8, false, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "#E2",
          getCoralPoseBasedOnTag(9, true, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "#E3",
          getCoralPoseBasedOnTag(9, true, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "#E4",
          getCoralPoseBasedOnTag(9, true, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "#F2",
          getCoralPoseBasedOnTag(9, false, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "#F3",
          getCoralPoseBasedOnTag(9, false, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "#F4",
          getCoralPoseBasedOnTag(9, false, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "#G2",
          getCoralPoseBasedOnTag(10, true, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "#G3",
          getCoralPoseBasedOnTag(10, true, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "#G4",
          getCoralPoseBasedOnTag(10, true, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "#H2",
          getCoralPoseBasedOnTag(10, false, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "#H3",
          getCoralPoseBasedOnTag(10, false, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "#H4",
          getCoralPoseBasedOnTag(10, false, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "#I2",
          getCoralPoseBasedOnTag(11, true, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "#I3",
          getCoralPoseBasedOnTag(11, true, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "#I4",
          getCoralPoseBasedOnTag(11, true, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "#J2",
          getCoralPoseBasedOnTag(11, false, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "#J3",
          getCoralPoseBasedOnTag(11, false, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "#J4",
          getCoralPoseBasedOnTag(11, false, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "#K2",
          getCoralPoseBasedOnTag(6, true, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "#K3",
          getCoralPoseBasedOnTag(6, true, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "#K4",
          getCoralPoseBasedOnTag(6, true, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));

      CORAL_POSES.put(
          "#L2",
          getCoralPoseBasedOnTag(6, false, ReefHeight.L2.heightMeter, ReefHeight.L2.pitchRad));
      CORAL_POSES.put(
          "#L3",
          getCoralPoseBasedOnTag(6, false, ReefHeight.L3.heightMeter, ReefHeight.L3.pitchRad));
      CORAL_POSES.put(
          "#L4",
          getCoralPoseBasedOnTag(6, false, ReefHeight.L4.heightMeter, ReefHeight.L4.pitchRad));
    }

    public static final double AlGAE_ADJUST_X = -GamePiece.Algae.DIAMETER / 2.0 + 0.045;
    public static final double AlGAE_HIGH_HEIGHT = 1.115 + GamePiece.Algae.DIAMETER / 2.0;
    public static final double AlGAE_LOW_HEIGHT = 0.711 + GamePiece.Algae.DIAMETER / 2.0;
    public static final Pose3d[] AlGAE_POSES =
        new Pose3d[] {
          getAlgaePoseBasedOnTag(18, AlGAE_HIGH_HEIGHT),
          getAlgaePoseBasedOnTag(17, AlGAE_LOW_HEIGHT),
          getAlgaePoseBasedOnTag(22, AlGAE_HIGH_HEIGHT),
          getAlgaePoseBasedOnTag(21, AlGAE_LOW_HEIGHT),
          getAlgaePoseBasedOnTag(20, AlGAE_HIGH_HEIGHT),
          getAlgaePoseBasedOnTag(19, AlGAE_LOW_HEIGHT),
          getAlgaePoseBasedOnTag(7, AlGAE_HIGH_HEIGHT),
          getAlgaePoseBasedOnTag(8, AlGAE_LOW_HEIGHT),
          getAlgaePoseBasedOnTag(9, AlGAE_HIGH_HEIGHT),
          getAlgaePoseBasedOnTag(10, AlGAE_LOW_HEIGHT),
          getAlgaePoseBasedOnTag(11, AlGAE_HIGH_HEIGHT),
          getAlgaePoseBasedOnTag(6, AlGAE_LOW_HEIGHT)
        };

    private static Pose2d getScorePoseBasedOnTag(int id, double adjustY) {
      return getPoseBasedOnTag(id, SCORE_ADJUST_X, adjustY, Rotation2d.kZero);
    }

    private static Pose2d getScorePoseBasedOnTag(int id, double adjustY, Rotation2d adjustHeading) {
      return getPoseBasedOnTag(id, SCORE_ADJUST_X, adjustY, adjustHeading);
    }

    private static Pose3d getCoralPoseBasedOnTag(
        int id, boolean isLeft, double height, double pitchRad) {
      return getPoseBasedOnTag(
          id, CORAL_ADJUST_X, CORAL_ADJUST_Y * (isLeft ? -1.0 : 1.0), height, -pitchRad);
    }

    private static Pose3d getTroughCoralPoseBasedOnTag(
        int id, boolean isLeft, double height, double diag) {
      var tagPose2d = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(id).get().toPose2d();
      var shiftedPose2d =
          tagPose2d.transformBy(
              new Transform2d(diag, CORAL_ADJUST_Y * (isLeft ? -1.0 : 1.0), new Rotation2d()));
      return new Pose3d(
          shiftedPose2d.getX(),
          shiftedPose2d.getY(),
          height,
          new Rotation3d(
              0.0,
              0.0,
              tagPose2d.getRotation().getRadians()
                  - (isLeft ? 1.0 : -1.0) * Rotation2d.kCCW_90deg.getRadians()));
    }

    private static Pose3d getAlgaePoseBasedOnTag(int id, double height) {
      return getPoseBasedOnTag(id, AlGAE_ADJUST_X, 0.0, height, 0.0);
    }

    private static Pose2d getPoseBasedOnTag(
        int id, double adjustX, double adjustY, Rotation2d adjustHeading) {
      var tagPose = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(id).get().toPose2d();
      var shiftedPose = tagPose.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()));
      return new Pose2d(
          shiftedPose.getTranslation(), shiftedPose.getRotation().rotateBy(adjustHeading));
    }

    private static Pose3d getPoseBasedOnTag(
        int id, double adjustX, double adjustY, double height, double pitchRad) {
      var tagPose2d = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(id).get().toPose2d();
      var shiftedPose2d =
          tagPose2d.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()));
      return new Pose3d(
          shiftedPose2d.getX(),
          shiftedPose2d.getY(),
          height,
          new Rotation3d(0.0, pitchRad, tagPose2d.getRotation().getRadians()));
    }
  }

  public static class Processor {
    public static final double FACE_LENGTH = 0.7112; // 28 inches
    public static final Pose2d CENTER =
        new Pose2d(17.548 - 11.56081, 1.032, Rotation2d.fromDegrees(270));
    public static final Pose3d ALGAE_POSE =
        new Pose3d(
            CENTER.getX(),
            CENTER.getY() - 1.032 - 0.32,
            GamePiece.Algae.DIAMETER / 2.0 + 0.1,
            new Rotation3d());
    private static final Pose2d SCORE_POSE = // TODO: adjust this based on the field
        new Pose2d(CENTER.getX(), CENTER.getY() - 0.48, CENTER.getRotation().plus(Rotation2d.kPi));

    public static Pose2d getScorePose() {
      if (!AllianceFlipUtil.shouldFlip()) {
        return SCORE_POSE;
      } else {
        return AllianceFlipUtil.mirror(SCORE_POSE);
      }
    }

    public static final Pose3d[] SCORABLE_POSES;

    public static Boolean closestRobotSide(Pose2d pose) {
      Pose2d tmpPose = !AllianceFlipUtil.inBlueHalf(pose) ? AllianceFlipUtil.mirror(pose) : pose;
      return tmpPose.getRotation().getCos() < 0;
    }

    static {
      SCORABLE_POSES =
          new Pose3d[] {
            Field.Processor.ALGAE_POSE,
            Field.Processor.ALGAE_POSE.plus(new Transform3d(0.435, 0, -0.05, new Rotation3d())),
            Field.Processor.ALGAE_POSE
                .plus(new Transform3d(0.435, 0, -0.05, new Rotation3d()))
                .plus(new Transform3d(0.435, 0, -0.05, new Rotation3d())),
            Field.Processor.ALGAE_POSE
                .plus(new Transform3d(0.435, 0, -0.05, new Rotation3d()))
                .plus(new Transform3d(0.435, 0, -0.05, new Rotation3d()))
                .plus(new Transform3d(0.435, 0, -0.05, new Rotation3d())),
            AllianceFlipUtil.mirror(Field.Processor.ALGAE_POSE),
            AllianceFlipUtil.mirror(
                Field.Processor.ALGAE_POSE.plus(
                    new Transform3d(0.435, 0, -0.05, new Rotation3d()))),
            AllianceFlipUtil.mirror(
                Field.Processor.ALGAE_POSE
                    .plus(new Transform3d(0.435, 0, -0.05, new Rotation3d()))
                    .plus(new Transform3d(0.435, 0, -0.05, new Rotation3d()))),
            AllianceFlipUtil.mirror(
                Field.Processor.ALGAE_POSE
                    .plus(new Transform3d(0.435, 0, -0.05, new Rotation3d()))
                    .plus(new Transform3d(0.435, 0, -0.05, new Rotation3d()))
                    .plus(new Transform3d(0.435, 0, -0.05, new Rotation3d())))
          };
    }
  }

  public static final Pose3d[] PRESET_CORAL_POSES, PRESET_ALGAE_POSES;

  static {
    final var BLUE_X = 1.222391128540039;
    final var RED_X = 16.335620880126953;
    final var BLUE_DROPPED_X = 1.3782422065734863;
    final var RED_DROPPED_X = LENGTH - 1.3782422065734863;

    final var LOW_Y = 2.191911458969116;
    final var MID_Y = 4.0191121101379395;
    final var HIGH_Y = 5.851738929748535;
    final var TOP_DROPPED_Y = 1.0819228887557983;
    final var BOTTOM_DROPPED_Y = WIDTH - 1.0819228887557983;

    final var Z = GamePiece.Coral.LENGTH / 2.0;

    final var ROTATION = new Rotation3d(0.0, Math.PI / 2.0, 0.0);

    PRESET_CORAL_POSES =
        new Pose3d[] {
          new Pose3d(BLUE_X, LOW_Y, Z, ROTATION),
          new Pose3d(BLUE_X, MID_Y, Z, ROTATION),
          new Pose3d(BLUE_X, HIGH_Y, Z, ROTATION),
          new Pose3d(
              BLUE_DROPPED_X, TOP_DROPPED_Y, GamePiece.Coral.DIAMETER / 2.0, new Rotation3d()),
          new Pose3d(
              BLUE_DROPPED_X, BOTTOM_DROPPED_Y, GamePiece.Coral.DIAMETER / 2.0, new Rotation3d()),
          new Pose3d(RED_X, LOW_Y, Z, ROTATION),
          new Pose3d(RED_X, MID_Y, Z, ROTATION),
          new Pose3d(RED_X, HIGH_Y, Z, ROTATION),
          new Pose3d(
              RED_DROPPED_X, TOP_DROPPED_Y, GamePiece.Coral.DIAMETER / 2.0, new Rotation3d()),
          new Pose3d(
              RED_DROPPED_X, BOTTOM_DROPPED_Y, GamePiece.Coral.DIAMETER / 2.0, new Rotation3d()),
        };

    PRESET_ALGAE_POSES =
        new Pose3d[] {
          new Pose3d(BLUE_X, LOW_Y, GamePiece.Algae.DIAMETER / 2.0 + Coral.LENGTH, ROTATION),
          new Pose3d(BLUE_X, MID_Y, GamePiece.Algae.DIAMETER / 2.0 + Coral.LENGTH, ROTATION),
          new Pose3d(BLUE_X, HIGH_Y, GamePiece.Algae.DIAMETER / 2.0 + Coral.LENGTH, ROTATION),
          new Pose3d(RED_X, LOW_Y, GamePiece.Algae.DIAMETER / 2.0 + Coral.LENGTH, ROTATION),
          new Pose3d(RED_X, MID_Y, GamePiece.Algae.DIAMETER / 2.0 + Coral.LENGTH, ROTATION),
          new Pose3d(RED_X, HIGH_Y, GamePiece.Algae.DIAMETER / 2.0 + Coral.LENGTH, ROTATION)
        };
  }

  public enum ReefHeight {
    L4(Units.inchesToMeters(72.0), Units.degreesToRadians(90.0)),
    L3(Units.inchesToMeters(47.625), Units.degreesToRadians(35.0)),
    L2(Units.inchesToMeters(31.875), Units.degreesToRadians(35.0)),
    L11(Units.inchesToMeters(20.0), Units.degreesToRadians(0.0)),
    L12(Units.inchesToMeters(19), Units.degreesToRadians(0.0));

    ReefHeight(double heightMeter, double pitchRad) {
      this.heightMeter = heightMeter;
      this.pitchRad = pitchRad; // in degrees
    }

    public final double heightMeter;
    public final double pitchRad;
  }

  public static final FieldType FIELD_TYPE = FieldType.ANDYMARK;

  @Getter
  @RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    private final String jsonFolder;
  }

  public static final AprilTagLayoutType APRILTAG_LAYOUT = AprilTagLayoutType.NO_BARGE;
  public static final int APRILTAG_COUNT = 22;

  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL("2025-official"),
    NO_BARGE("2025-no-barge"),
    BLUE_REEF("2025-blue-reef"),
    RED_REEF("2025-red-reef");

    AprilTagLayoutType(String name) {
      try {
        layout =
            new AprilTagFieldLayout(
                Path.of(
                    Filesystem.getDeployDirectory().getPath(),
                    "apriltags",
                    FIELD_TYPE.getJsonFolder(),
                    name + ".json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException("Failed to serialize AprilTag layout JSON " + this);
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }
}
