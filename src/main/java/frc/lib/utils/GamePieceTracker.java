package frc.lib.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.math.GeomUtil;
import frc.lib.math.PolygonUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class GamePieceTracker {
  private final String name;
  private final TimeBuffer<Pose2d[]> tracked;

  public GamePieceTracker(String name, double historySizeSec) {
    this.name = name;
    tracked = new TimeBuffer<>(historySizeSec);
  }

  public void add(Pose2d[] gamePieces, double timestamp) {
    tracked.add(gamePieces, timestamp);
  }

  public Optional<Pose2d> getBest(Pose2d robotPose, Translation2d[] areaPolygonVertices) {
    var positionsBuffer = tracked.getBuffer();
    if (positionsBuffer.isEmpty()) {
      return Optional.empty();
    }

    // for better visualization in advantage scope
    var areaPolygonVerticesList = new ArrayList<>(List.of(areaPolygonVertices));
    areaPolygonVerticesList.add(areaPolygonVertices[0]);

    Logger.recordOutput(
        "GamePieceTracker/SearchArea" + name,
        areaPolygonVerticesList.toArray(Translation2d[]::new));

    for (var timestamp : positionsBuffer.descendingKeySet()) {
      var positions = positionsBuffer.get(timestamp);
      var goodGamePieceInField =
          Arrays.stream(positions)
              .filter(
                  gamePieceInField ->
                      PolygonUtil.isInPolygon(
                          gamePieceInField.getTranslation(), areaPolygonVertices))
              .min(
                  Comparator.comparingDouble(
                      gamePieceInField ->
                          gamePieceInField
                              .getTranslation()
                              .getDistance(robotPose.getTranslation())));

      if (goodGamePieceInField.isPresent()) {
        Logger.recordOutput(
            "GamePieceTracker/Best" + name, new Pose2d[] {goodGamePieceInField.get()});
        return goodGamePieceInField;
      }
    }

    Logger.recordOutput("GamePieceTracker/Best" + name, new Pose2d[] {});
    return Optional.empty();
  }

  public Optional<Pose2d> getBest(
      Pose2d robotPose,
      Translation2d searchLineInRobot,
      double xSearchDistanceMeter,
      double ySearchDistanceMeter) {
    var positionsBuffer = tracked.getBuffer();
    if (positionsBuffer.isEmpty()) {
      return Optional.empty();
    }

    var searchLineInRobotPose = searchLineInRobot.toRotation2d().toPose2d();
    for (var timestamp : positionsBuffer.descendingKeySet()) {
      var positions = positionsBuffer.get(timestamp);
      var goodGamePieceInField =
          Arrays.stream(positions)
              .filter(
                  gamePieceInField -> {
                    var gamePieceInSearchLine =
                        gamePieceInField.relativeTo(robotPose).relativeTo(searchLineInRobotPose);
                    var isRobotFaceToGamePiece = gamePieceInSearchLine.getX() > 0.0;
                    var isXCloseEnough =
                        Math.abs(gamePieceInSearchLine.getX()) <= xSearchDistanceMeter;
                    var isYCloseEnough =
                        Math.abs(gamePieceInSearchLine.getY()) <= ySearchDistanceMeter;

                    return isRobotFaceToGamePiece && isXCloseEnough && isYCloseEnough;
                  })
              .min(
                  Comparator.comparingDouble(
                      gamePieceInField ->
                          gamePieceInField
                              .relativeTo(robotPose)
                              .relativeTo(searchLineInRobotPose)
                              .getY()));

      if (goodGamePieceInField.isPresent()) {
        Logger.recordOutput(
            "GamePieceTracker/Best" + name, new Pose2d[] {goodGamePieceInField.get()});
        return goodGamePieceInField;
      }
    }

    Logger.recordOutput("GamePieceTracker/Best" + name, new Pose2d[] {});
    return Optional.empty();
  }

  public Pose2d[] getAllTracked() {
    var gamePieceBuffer = tracked.getBuffer().values();
    var all = new ArrayList<Pose2d>();
    for (var gamePiece : gamePieceBuffer) {
      all.addAll(Arrays.asList(gamePiece));
    }
    return all.toArray(Pose2d[]::new);
  }
}
