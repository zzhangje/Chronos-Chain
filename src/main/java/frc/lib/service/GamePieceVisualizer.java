package frc.lib.service;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.interfaces.VirtualSubsystem;
import frc.lib.math.GeomUtil;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class GamePieceVisualizer extends VirtualSubsystem {
  private final String name;

  // FIXME: reset gamepiece when reset() is called
  private final Pose3d[] scorableGamePiecePose;
  @Getter private final Pose3d[] pickableGamePiecePose;
  private final Pose3d[] scoredGamePiecePose;
  @Getter private int hasGamePieceNums = 0;
  private int scoredGamePieceNums = 0;

  private final double pickableMaxDistance;
  private final double scorableMaxDistance;

  @Override
  public void periodic() {
    Logger.recordOutput("Visualization/" + name + "/hasGamePiece", hasGamePieceNums);
    Logger.recordOutput("Visualization/" + name + "/scoredGamePieceNums", scoredGamePieceNums);

    Logger.recordOutput("Visualization/" + name + "/pickableGamePiecePose", pickableGamePiecePose);
    Logger.recordOutput("Visualization/" + name + "/scoredGamePiecePose", scoredGamePiecePose);
    Logger.recordOutput("Visualization/" + name + "/scorableGamePiecePose", scorableGamePiecePose);
  }

  /**
   * Tries to pick a game piece at the specified pose.
   *
   * @param pose The pose of the game piece to pick.
   * @return true if the game piece was successfully picked, false otherwise.
   */
  public boolean tryPick(Pose3d pose) {
    Logger.recordOutput("Visualization/" + name + "/tryPickPose", pose);
    double minDistance = Double.POSITIVE_INFINITY;
    int minDistanceIndex = 0;
    for (int i = 0; i < pickableGamePiecePose.length; i++) {
      double distance = pickableGamePiecePose[i].getDistance(pose);
      if (distance < minDistance) {
        minDistance = distance;
        minDistanceIndex = i;
      }
    }
    if (minDistance < pickableMaxDistance) {
      pickableGamePiecePose[minDistanceIndex] =
          new Pose3d(0x3f3f3f3f, 0x3f3f3f3f, 0x3f3f3f3f, new Rotation3d());
      hasGamePieceNums++;
      return true;
    }

    return false;
  }

  /**
   * Tries to eject a game piece at the specified pose.
   *
   * @param pose The pose where the game piece should be ejected.
   * @return true if the game piece was successfully ejected, false otherwise.
   */
  public boolean tryEject(Pose3d pose) {
    if (hasGamePieceNums > 0) {
      hasGamePieceNums--;
      // FIXME: make it pickable instead simply remove
    }
    return true;
  }

  /**
   * Tries to score a game piece at the specified pose.
   *
   * @param pose The pose where the game piece should be scored.
   * @return true if the game piece was successfully scored, false otherwise.
   */
  public boolean tryScore(Pose3d pose) {
    Logger.recordOutput("Visualization/" + name + "/tryScorePose", pose);
    if (hasGamePieceNums > 0) {
      double minDistance = Double.POSITIVE_INFINITY;
      int minDistanceIndex = -1;
      double minScoredDistance = Double.POSITIVE_INFINITY;
      int minScoredDistanceIndex = -1;

      // Iterate through all positions
      for (int i = 0; i < scorableGamePiecePose.length; i++) {
        // Check if this position has already been scored
        boolean isScored = false;
        for (int j = 0; j < scoredGamePieceNums; j++) {
          if (scoredGamePiecePose[j].equals(scorableGamePiecePose[i])) {
            isScored = true;
            break;
          }
        }

        double distance = scorableGamePiecePose[i].getDistance(pose);

        // Record the closest distance among unscored positions
        if (!isScored && distance < minDistance) {
          minDistance = distance;
          minDistanceIndex = i;
        }

        // Record the closest distance among scored positions
        if (isScored && distance < minScoredDistance) {
          minScoredDistance = distance;
          minScoredDistanceIndex = i;
        }
      }

      // If an unscored position is found within threshold, score at that position
      if (minDistanceIndex != -1 && minDistance < scorableMaxDistance) {
        scoredGamePiecePose[scoredGamePieceNums] = scorableGamePiecePose[minDistanceIndex];
        scoredGamePieceNums++;
        hasGamePieceNums--;
        return true;
      }
      // If all positions are scored, score at the closest scored position
      else if (minScoredDistanceIndex != -1 && minScoredDistance < scorableMaxDistance) {
        scoredGamePiecePose[scoredGamePieceNums] = scorableGamePiecePose[minScoredDistanceIndex];
        scoredGamePieceNums++;
        hasGamePieceNums--;
        return true;
      }
    }
    return false;
  }

  public GamePieceVisualizer(
      String name,
      double maxPickableDistance,
      double maxScorableDistance,
      Pose3d[] pickableGamePiecePose,
      Pose3d[] scorableGamePiecePose,
      int hasGamePieceNums) {
    this.name = name;
    this.pickableMaxDistance = maxPickableDistance;
    this.scorableMaxDistance = maxScorableDistance;
    this.scorableGamePiecePose = scorableGamePiecePose;
    this.pickableGamePiecePose = pickableGamePiecePose;
    this.scoredGamePiecePose = new Pose3d[pickableGamePiecePose.length];
    for (int i = 0; i < pickableGamePiecePose.length; i++) {
      scoredGamePiecePose[i] = new Pose3d(0x3f3f3f3f, 0x3f3f3f3f, 0x3f3f3f3f, new Rotation3d());
    }
    this.hasGamePieceNums = hasGamePieceNums;
  }
}
