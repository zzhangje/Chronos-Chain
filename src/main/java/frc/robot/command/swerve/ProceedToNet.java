package frc.robot.command.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystem.swerve.Swerve;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ProceedToNet extends Command {
  private static final LoggedTunableNumber translationKp =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToNet/TranslationKp", 1.9);
  private static final LoggedTunableNumber translationKd =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToNet/TranslationKd", 0.1);
  private static final LoggedTunableNumber translationToleranceMeter =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToNet/TranslationToleranceMeter", 0.12);
  private static final LoggedTunableNumber maxTranslationVelMeterPerSec =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToNet/MaxTranslationVelMeterPerSec", 3.1);

  private static final LoggedTunableNumber rotationKp =
      new LoggedTunableNumber(Constants.DebugGroup.SWERVE, "Swerve/ProceedToNet/RotationKp", 4.0);
  private static final LoggedTunableNumber rotationKd =
      new LoggedTunableNumber(Constants.DebugGroup.SWERVE, "Swerve/ProceedToNet/RotationKd", 0.1);
  private static final LoggedTunableNumber rotationToleranceDegree =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToNet/RotationToleranceDegree", 5.0);
  private static final LoggedTunableNumber maxRotationVelDegreePerSec =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToNet/MaxRotationDegreePerSec", 360.0);

  private final Swerve swerve;
  private final Supplier<Pose2d> goalPoseSupplier;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  private boolean hasDone = false;
  private double translationErrorMeter = Double.POSITIVE_INFINITY;
  private boolean hasHeadingAtGoal = false;

  public ProceedToNet(Swerve swerve, Supplier<Pose2d> goalPoseSupplier) {
    this.swerve = swerve;
    this.goalPoseSupplier = goalPoseSupplier;

    xController = new PIDController(translationKp.get(), 0.0, translationKd.get());
    yController = new PIDController(translationKp.get(), 0.0, translationKd.get());
    rotationController = new PIDController(rotationKp.get(), 0.0, rotationKd.get());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    var goalPose = goalPoseSupplier.get();
    var currentPose = RobotState.getOdometry().getEstimatedPose();

    Logger.recordOutput("Swerve/ProceedToNet/GoalPose", goalPose);

    var xFeedback = xController.calculate(currentPose.getX(), goalPose.getX());
    var yFeedback = yController.calculate(currentPose.getY(), goalPose.getY());
    var rotationFeedback =
        rotationController.calculate(
            MathUtil.angleModulus(currentPose.getRotation().getRadians()),
            MathUtil.angleModulus(goalPose.getRotation().getRadians()));

    translationErrorMeter = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    Logger.recordOutput("Swerve/ProceedToNet/TranslationErrorMeter", translationErrorMeter);

    var rotationErrorDegree = currentPose.getRotation().minus(goalPose.getRotation()).getDegrees();
    Logger.recordOutput("Swerve/ProceedToNet/RotationErrorDegree", rotationErrorDegree);

    hasHeadingAtGoal = Math.abs(rotationErrorDegree) <= rotationToleranceDegree.get();
    hasDone =
        Math.abs(currentPose.getTranslation().getDistance(goalPose.getTranslation()))
                <= translationToleranceMeter.get()
            && hasHeadingAtGoal;

    if (!hasDone) {
      var translationOutputScalar =
          hasHeadingAtGoal ? 1.0 : 1.0 - Math.abs(rotationErrorDegree) / 180.0;
      Logger.recordOutput("Swerve/ProceedToNet/TranslationOutputScalar", translationOutputScalar);

      var maxTranslationVelMeterPerSecVal = maxTranslationVelMeterPerSec.get();
      var maxRotationVelRadPerSec = Units.degreesToRadians(maxRotationVelDegreePerSec.get());

      ChassisSpeeds chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              MathUtil.clamp(
                  xFeedback * translationOutputScalar,
                  -maxTranslationVelMeterPerSecVal,
                  maxTranslationVelMeterPerSecVal),
              MathUtil.clamp(
                  yFeedback * translationOutputScalar,
                  -maxTranslationVelMeterPerSecVal,
                  maxTranslationVelMeterPerSecVal),
              MathUtil.clamp(rotationFeedback, -maxRotationVelRadPerSec, maxRotationVelRadPerSec),
              currentPose.getRotation());

      swerve.setGoalVel(chassisSpeeds);
    } else {
      swerve.setGoalVel(new ChassisSpeeds());
    }
  }

  @Override
  public boolean isFinished() {
    return hasDone;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setGoalVel(new ChassisSpeeds());
  }

  public boolean hasHeadingAtGoal() {
    return hasHeadingAtGoal;
  }

  public boolean hasDistanceWithin(double toleranceMeter) {
    return Math.abs(translationErrorMeter) <= toleranceMeter;
  }
}
