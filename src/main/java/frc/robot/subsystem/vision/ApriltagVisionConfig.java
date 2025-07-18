package frc.robot.subsystem.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

class ApriltagVisionConfig {
  static final Pose3d CASUAL_MID_LEFT_IN_ROBOT =
      new Pose3d(
          0.01487,
          0.22715,
          0.95679,
          new Rotation3d(0.0, Units.degreesToRadians(-57.0), Units.degreesToRadians(85.0)));
  static final Pose3d CASUAL_MID_RIGHT_IN_ROBOT =
      new Pose3d(
          0.01487,
          -0.22715,
          0.95679,
          new Rotation3d(0.0, Units.degreesToRadians(-57.0), Units.degreesToRadians(-85.0)));
  static final Pose3d REEF_BACK_LEFT_IN_ROBOT =
      new Pose3d(
          -0.0099 - 0.06,
          0.21620,
          0.20304,
          new Rotation3d(0.0, Units.degreesToRadians(14.0), Units.degreesToRadians(105.0)));
  static final Pose3d REEF_BACK_RIGHT_IN_ROBOT =
      new Pose3d(
          -0.0099 - 0.06,
          -0.21620,
          0.20304,
          new Rotation3d(0.0, Units.degreesToRadians(14.0), Units.degreesToRadians(-105.0)));
  static final double FIELD_BORDER_THRESHOLD_METER = 0.5;
  static final double ROBOT_POSE_Z_THRESHOLD_METER = 0.5;
  static final double XY_STD_DEV_COEFF = 0.005;
  static final double THETA_STD_DEV_COEFF = 0.01;
  static final double MAX_ALLOWABLE_AMBIGUITY = 0.4;
}
