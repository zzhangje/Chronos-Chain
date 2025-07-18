package frc.lib.math;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Geometry utilities for working with translations, rotations, transforms, and poses. */
public class GeomUtil {
  /**
   * Creates a pure translating transform
   *
   * @param translation The translation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure translating transform
   *
   * @param x The x coordinate of the translation
   * @param y The y coordinate of the translation
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(double x, double y) {
    return new Transform2d(x, y, new Rotation2d());
  }

  /**
   * Creates a pure rotating transform
   *
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Rotation2d rotation) {
    return new Transform2d(new Translation2d(), rotation);
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Pose2d pose) {
    return pose.minus(new Pose2d());
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Pose2d inverse(Pose2d pose) {
    return new Pose2d().relativeTo(pose);
  }

  /**
   * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
   * chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose2d toPose2d(Transform2d transform) {
    return new Pose2d().plus(transform);
  }

  /**
   * Creates a pure translated pose
   *
   * @param translation The translation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d toPose2d(Translation2d translation) {
    return new Pose2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure rotated pose
   *
   * @param rotation The rotation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d toPose2d(Rotation2d rotation) {
    return new Pose2d(new Translation2d(), rotation);
  }

  /**
   * Multiplies a twist by a scaling factor
   *
   * @param twist The twist to multiply
   * @param factor The scaling factor for the twist components
   * @return The new twist
   */
  public static Twist2d multiply(Twist2d twist, double factor) {
    return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
  }

  public static Transform2d exp(Twist2d twist) {
    return new Pose2d().exp(twist).minus(new Pose2d());
  }

  public static Twist2d log(Transform2d transform) {
    return new Pose2d().log(new Pose2d().plus(transform));
  }

  /**
   * Converts a Translation3d to a Rotation3d
   *
   * @param translation The translation that will represent the rotation
   * @return The resulting direction
   */
  public static Rotation3d toRotation3d(Translation3d translation) {
    return new Rotation3d(
        0.0,
        -Math.atan(
            translation.getZ() / (Math.hypot(translation.getX(), translation.getY()) + 1e-16)),
        Math.atan(translation.getY() / (translation.getX() + 1e-16)));
  }

  /**
   * Converts a Pose3d to a Transform3d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform3d toTransform3d(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
   * chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose3d toPose3d(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Converts a Pose2d to a Pose3d to be used as a position or as the start of a kinematic chain
   *
   * @param pose The pose2d that will represent the pose3d
   * @return The resulting pose
   */
  public static Pose3d toPose3d(Pose2d pose) {
    return new Pose3d(pose);
  }

  /**
   * Converts a ChassisSpeeds to a Twist2d by extracting two dimensions (Y and Z). chain
   *
   * @param speeds The original translation
   * @return The resulting translation
   */
  public static Twist2d toTwist2d(ChassisSpeeds speeds) {
    return new Twist2d(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  /**
   * Converts a Translation2d to a Rotation2d by extracting two dimensions (X and Y). chain
   *
   * @param translation The original translation
   * @return The resulting direction
   */
  public static Rotation2d toRotation2d(Translation2d translation) {
    return translation.getNorm() < 1e-6
        ? new Rotation2d()
        : new Rotation2d(translation.getX(), translation.getY());
  }

  /**
   * Converts a Twist2d to a Translation2d by extracting two dimensions (X and Y). chain
   *
   * @param twist The original twist
   * @return The resulting translation
   */
  public static Translation2d toTranslation2d(Twist2d twist) {
    return new Translation2d(twist.dx, twist.dy);
  }

  /**
   * Converts a Translation2d to a Translation3d by extracting two dimensions (X and Y) and zero Z.
   * chain
   *
   * @param translation The original translation
   * @return The resulting translation
   */
  public static Translation3d toTranslation3d(Translation2d translation) {
    return new Translation3d(translation);
  }

  /**
   * Creates a new pose from an existing one using a different translation value.
   *
   * @param pose The original pose
   * @param translation The new translation to use
   * @return The new pose with the new translation and original rotation
   */
  public static Pose2d withTranslation(Pose2d pose, Translation2d translation) {
    return new Pose2d(translation, pose.getRotation());
  }

  /**
   * Creates a new pose from an existing one using a different rotation value.
   *
   * @param pose The original pose
   * @param rotation The new rotation to use
   * @return The new pose with the original translation and new rotation
   */
  public static Pose2d withRotation(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getTranslation(), rotation);
  }

  public static double getDistance(Pose3d pose1, Pose3d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }
}
