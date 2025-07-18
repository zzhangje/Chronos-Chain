package frc.robot.command.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.lib.math.GeomUtil;
import frc.reefscape.Field;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystem.swerve.Swerve;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ProceedToProcessor extends Command {
  private static final LoggedTunableNumber maxLineupShiftingYMeter =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToProcessor/MaxLineupShiftingYMeter", 0.5);
  private static final LoggedTunableNumber maxLineupShiftingXMeter =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToProcessor/MaxLineupShiftingXMeter", 1.5);

  private static final LoggedTunableNumber translationKp =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToProcessor/TranslationKp", 7.2);
  private static final LoggedTunableNumber translationKd =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToProcessor/TranslationKd", 1.8);
  private static final LoggedTunableNumber translationToleranceMeter =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToProcessor/TranslationToleranceMeter", 0.15);
  private static final LoggedTunableNumber maxTranslationVelMeterPerSec =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE,
          "Swerve/ProceedToProcessor/MaxTranslationVelMeterPerSec",
          3.8);

  private static final LoggedTunableNumber rotationKp =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToProcessor/RotationKp", 4.0);
  private static final LoggedTunableNumber rotationKd =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToProcessor/RotationKd", 0.0);
  private static final LoggedTunableNumber rotationToleranceDegree =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToProcessor/RotationToleranceDegree", 12.5);
  private static final LoggedTunableNumber maxRotationVelDegreePerSec =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToProcessor/MaxRotationDegreePerSec", 360.0);

  private final Swerve swerve;
  private final Supplier<Pose2d> goalPoseSupplier;
  private final Supplier<Rotation2d> alignRotationSupplier;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  private boolean hasDone = false;
  private boolean hasHeadingAtGoal = false;
  private boolean hasPositionAtGoal = false;
  private double translationErrorMeter = Double.POSITIVE_INFINITY;

  public ProceedToProcessor(
      Swerve swerve,
      Supplier<Pose2d> goalPoseSupplier,
      Supplier<Rotation2d> alignRotationSupplier) {
    this.swerve = swerve;
    this.goalPoseSupplier = goalPoseSupplier;
    this.alignRotationSupplier = alignRotationSupplier;

    xController = new PIDController(translationKp.get(), 0.0, translationKd.get());
    yController = new PIDController(translationKp.get(), 0.0, translationKd.get());
    rotationController = new PIDController(rotationKp.get(), 0.0, rotationKd.get());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    var goalPose =
        new Pose2d(getShiftedGoalPose().getTranslation(), goalPoseSupplier.get().getRotation());
    var currentPose = getCurrentPose();

    Logger.recordOutput("Swerve/ProceedToProcessor/GoalPose", goalPose);

    var xFeedback = xController.calculate(currentPose.getX(), goalPose.getX());
    var yFeedback = yController.calculate(currentPose.getY(), goalPose.getY());
    var rotationFeedback =
        rotationController.calculate(
            MathUtil.angleModulus(currentPose.getRotation().getRadians()),
            MathUtil.angleModulus(goalPose.getRotation().getRadians()));

    translationErrorMeter = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    Logger.recordOutput("Swerve/ProceedToProcessor/TranslationErrorMeter", translationErrorMeter);

    var rotationErrorDegree = currentPose.getRotation().minus(goalPose.getRotation()).getDegrees();
    Logger.recordOutput("Swerve/ProceedToProcessor/RotationErrorDegree", rotationErrorDegree);

    hasHeadingAtGoal = Math.abs(rotationErrorDegree) <= rotationToleranceDegree.get();
    hasPositionAtGoal =
        Math.abs(currentPose.getTranslation().getDistance(goalPose.getTranslation()))
            <= translationToleranceMeter.get();
    hasDone = hasPositionAtGoal && hasHeadingAtGoal;

    var translationOutputScalar =
        hasHeadingAtGoal ? 1.0 : 1.0 - Math.abs(rotationErrorDegree) / 180.0;
    Logger.recordOutput(
        "Swerve/ProceedToProcessor/TranslationOutputScalar", translationOutputScalar);

    var maxTranslationVelMeterPerSecVal = maxTranslationVelMeterPerSec.get();
    var maxRotationVelRadPerSec = Units.degreesToRadians(maxRotationVelDegreePerSec.get());

    var chassisSpeeds =
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
  }

  private Pose2d getShiftedGoalPose() {
    var goalPose = new Pose2d(goalPoseSupplier.get().getTranslation(), alignRotationSupplier.get());
    var currentPose = getCurrentPose();

    var flippedGoalPose =
        new Pose2d(goalPose.getTranslation(), goalPose.getRotation().rotateBy(Rotation2d.k180deg));

    var offset = currentPose.relativeTo(flippedGoalPose);
    var yDistance = Math.abs(offset.getY());
    var xDistance = Math.abs(offset.getX());
    var shiftXT =
        MathUtil.clamp(
            (yDistance / (Field.Reef.FACE_LENGTH * 2.0))
                + ((xDistance - 0.3) / (Field.Reef.FACE_LENGTH * 3.0)),
            0.0,
            1.0);
    var shiftYT = MathUtil.clamp(offset.getX() / Field.Reef.FACE_LENGTH, 0.0, 1.0);
    var flippedShiftedGoalPose =
        flippedGoalPose.transformBy(
            GeomUtil.toTransform2d(
                -shiftXT * maxLineupShiftingXMeter.get(),
                Math.copySign(shiftYT * maxLineupShiftingYMeter.get() * 0.8, offset.getY())));

    return new Pose2d(flippedShiftedGoalPose.getTranslation(), goalPose.getRotation());
  }

  private Pose2d getCurrentPose() {
    return RobotState.getOdometry().getEstimatedPose();
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

  public boolean hasPositionAtGoal() {
    return hasPositionAtGoal;
  }

  public boolean hasDistanceWithin(double toleranceMeter) {
    return Math.abs(
            getCurrentPose().getTranslation().getDistance(goalPoseSupplier.get().getTranslation()))
        <= toleranceMeter;
  }
}
