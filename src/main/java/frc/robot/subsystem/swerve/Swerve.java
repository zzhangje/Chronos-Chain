package frc.robot.subsystem.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class Swerve {

  public record WheeledObservation(
      double timestamp, SwerveModulePosition[] wheelPositions, Rotation2d yaw) {}
}
