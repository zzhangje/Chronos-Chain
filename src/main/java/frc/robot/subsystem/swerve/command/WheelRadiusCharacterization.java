package frc.robot.subsystem.swerve.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.Constants.DebugGroup;
import frc.robot.subsystem.swerve.Module;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.SwerveConfig;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class WheelRadiusCharacterization extends Command {
  private static final LoggedTunableNumber characterizationSpeedDegreePerSec =
      new LoggedTunableNumber(
          DebugGroup.SWERVE, "Swerve/WheelRadiusCharacterization/SpeedDegreePerSec", 360.0);
  private final DoubleSupplier gyroYawRadsSupplier;

  @RequiredArgsConstructor
  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int value;
  }

  private final Swerve swerve;
  private final Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double sumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  public WheelRadiusCharacterization(
      Swerve swerve, Supplier<Rotation2d> yawSupplier, Direction rotateDirection) {
    this.swerve = swerve;
    this.omegaDirection = rotateDirection;
    this.gyroYawRadsSupplier = () -> yawSupplier.get().getRadians();
    addRequirements(swerve);
    withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    setName("Swerve/Wheel Radius Characterization");
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    sumGyroYawRads = 0.0;

    startWheelPositions =
        Arrays.stream(swerve.getModules()).mapToDouble(Module::getDrivePositionRad).toArray();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    swerve.setGoalVel(
        new ChassisSpeeds(
            0.0,
            0.0,
            omegaLimiter.calculate(
                omegaDirection.value
                    * Units.degreesToRadians(characterizationSpeedDegreePerSec.get()))),
        false);

    var currentGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    sumGyroYawRads += MathUtil.angleModulus(currentGyroYawRads - lastGyroYawRads);
    lastGyroYawRads = currentGyroYawRads;

    var sumWheelPosition = 0.0;
    var wheelPositions =
        Arrays.stream(swerve.getModules()).mapToDouble(Module::getDrivePositionRad).toArray();
    for (int i = 0; i < 4; i++) {
      sumWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
    }
    var avgWheelPosition = sumWheelPosition / 4.0;

    currentEffectiveWheelRadius =
        (sumGyroYawRads * (SwerveConfig.WHEELBASE_DIAGONAL_METER / 2.0)) / avgWheelPosition;
    Logger.recordOutput("Swerve/WheelRadiusCharacterization/DrivePosition", avgWheelPosition);
    Logger.recordOutput("Swerve/WheelRadiusCharacterization/AccumGyroYawRads", sumGyroYawRads);
    Logger.recordOutput(
        "Swerve/WheelRadiusCharacterization/CurrentWheelRadiusMeters", currentEffectiveWheelRadius);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();

    if (sumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("No enough data for characterization");
    } else {
      System.out.println("Effective Wheel Radius: " + currentEffectiveWheelRadius + " meters");
    }
  }
}
