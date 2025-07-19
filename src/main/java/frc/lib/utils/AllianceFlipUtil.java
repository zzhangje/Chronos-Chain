package frc.lib.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.reefscape.Field;

public class AllianceFlipUtil {
  public static double mirrorX(double x) {
    return Field.LENGTH - x;
  }

  public static double applyX(double x) {
    return shouldFlip() ? mirrorX(x) : x;
  }

  public static double mirrorY(double y) {
    return Field.WIDTH - y;
  }

  public static double applyY(double y) {
    return shouldFlip() ? mirrorY(y) : y;
  }

  public static Translation2d mirror(Translation2d translation) {
    return new Translation2d(mirrorX(translation.getX()), mirrorY(translation.getY()));
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d mirror(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.kPi);
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d mirror(Pose2d pose) {
    return new Pose2d(mirror(pose.getTranslation()), mirror(pose.getRotation()));
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Pose3d mirror(Pose3d pose) {
    var translation2d = mirror(pose.getTranslation().toTranslation2d());
    return new Pose3d(
        new Translation3d(translation2d.getX(), translation2d.getY(), pose.getZ()),
        new Rotation3d(
            pose.getRotation().getX(),
            pose.getRotation().getY(),
            mirror(pose.getRotation().toRotation2d()).getRadians()));
  }

  public static Pose3d apply(Pose3d pose) {
    var translation2d = apply(pose.getTranslation().toTranslation2d());

    return shouldFlip()
        ? new Pose3d(
            new Translation3d(translation2d.getX(), translation2d.getY(), pose.getZ()),
            new Rotation3d(
                pose.getRotation().getX(),
                pose.getRotation().getY(),
                apply(pose.getRotation().toRotation2d()).getRadians()))
        : pose;
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  public static boolean inBlueHalf(Pose2d pose) {
    return pose.getX() < Field.LENGTH / 2.0;
  }

  public static boolean inRedHalf(Pose2d pose) {
    return pose.getX() > Field.LENGTH / 2.0;
  }
}
