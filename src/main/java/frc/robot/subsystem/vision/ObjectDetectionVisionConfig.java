package frc.robot.subsystem.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class ObjectDetectionVisionConfig {
  static final Pose3d FRONT_MID_IN_ROBOT =
      new Pose3d(
          0.122714,
          0.198081,
          0.878499,
          new Rotation3d(0.0, Units.degreesToRadians(35.0), Units.degreesToRadians(-5.0)));
}
