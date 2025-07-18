package frc.robot.command.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.lib.math.GeomUtil;
import frc.reefscape.Field;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystem.swerve.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ProceedToReef extends Command {
  private static final LoggedTunableNumber maxLineupShiftingYMeter =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToReef/MaxLineupShiftingYMeter", 1.75);
  private static final LoggedTunableNumber maxLineupShiftingXMeter =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToReef/MaxLineupShiftingXMeter", 1.25);
  private static final LoggedTunableNumber minLineupHesitatedShiftingXMeter =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE,
          "Swerve/ProceedToReef/MinLineupHesitatedShiftingXMeter",
          0.3);

  private static final LoggedTunableNumber translationKp =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToReef/TranslationKp", 4.25);
  private static final LoggedTunableNumber translationKd =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToReef/TranslationKd", 1.3);
  private static final LoggedTunableNumber translationToleranceMeter =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToReef/TranslationToleranceMeter", 0.07);
  private static final LoggedTunableNumber maxTranslationVelMeterPerSec =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToReef/MaxTranslationVelMeterPerSec", 3.8);

  private static final LoggedTunableNumber rotationKp =
      new LoggedTunableNumber(Constants.DebugGroup.SWERVE, "Swerve/ProceedToReef/RotationKp", 3.7);
  private static final LoggedTunableNumber rotationKd =
      new LoggedTunableNumber(Constants.DebugGroup.SWERVE, "Swerve/ProceedToReef/RotationKd", 0.3);
  private static final LoggedTunableNumber rotationToleranceDegree =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToReef/RotationToleranceDegree", 7.0);
  private static final LoggedTunableNumber maxRotationVelDegreePerSec =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/ProceedToReef/MaxRotationDegreePerSec", 540.0);

  private final Supplier<Pose2d> goalPoseSupplier;
  private final Supplier<Rotation2d> alignRotationSupplier;
  private final BooleanSupplier stopHesitationSignalSupplier;
  private final Swerve swerve;

  private final PIDController translationController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController rotationController = new PIDController(0.0, 0.0, 0.0);

  private boolean hasDone = false;
  private boolean hasHeadingAtGoal = false;
  private boolean needHesitation = true;

  public ProceedToReef(
      Supplier<Pose2d> goalPoseSupplier,
      Supplier<Rotation2d> alignRotationSupplier,
      BooleanSupplier stopHesitationSignalSupplier,
      Swerve swerve) {
    this.goalPoseSupplier = goalPoseSupplier;
    this.alignRotationSupplier = alignRotationSupplier;
    this.stopHesitationSignalSupplier = stopHesitationSignalSupplier;
    this.swerve = swerve;

    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
    setName("Swerve/AlignToReef");
  }

  @Override
  public void execute() {
    translationController.setP(translationKp.get());
    translationController.setD(translationKd.get());
    translationController.setTolerance(translationToleranceMeter.get());
    rotationController.setP(rotationKp.get());
    rotationController.setD(rotationKd.get());
    rotationController.setTolerance(rotationToleranceDegree.get());

    if (stopHesitationSignalSupplier.getAsBoolean()) {
      needHesitation = false;
    }

    var shiftedGoalPose =
        new Pose2d(getShiftedGoalPose().getTranslation(), goalPoseSupplier.get().getRotation());
    var currentPose = getCurrentPose();
    Logger.recordOutput("Swerve/ProceedToReef/GoalPose", shiftedGoalPose);

    var currentDistance =
        currentPose.getTranslation().getDistance(shiftedGoalPose.getTranslation());
    Logger.recordOutput("Swerve/ProceedToReef/TranslationErrorMeter", currentDistance);

    var translationDir =
        shiftedGoalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();

    var translationFeedback = translationController.calculate(0.0, currentDistance);

    var rotationErrorDegree =
        currentPose.getRotation().minus(shiftedGoalPose.getRotation()).getDegrees();
    Logger.recordOutput("Swerve/ProceedToReef/RotationErrorDegree", rotationErrorDegree);

    hasHeadingAtGoal = Math.abs(rotationErrorDegree) <= rotationToleranceDegree.get();
    hasDone =
        Math.abs(currentPose.getTranslation().getDistance(shiftedGoalPose.getTranslation()))
                <= translationToleranceMeter.get()
            && hasHeadingAtGoal;

    var translationOutputScalar =
        hasHeadingAtGoal ? 1.0 : 1.0 - Math.abs(rotationErrorDegree) / 180.0;
    Logger.recordOutput("Swerve/ProceedToReef/TranslationOutputScalar", translationOutputScalar);

    var translationVel =
        new Translation2d(
            MathUtil.clamp(
                translationFeedback * translationOutputScalar,
                -maxTranslationVelMeterPerSec.get(),
                maxTranslationVelMeterPerSec.get()),
            translationDir);
    var rotationVel =
        MathUtil.clamp(
            rotationController.calculate(
                MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                MathUtil.angleModulus(shiftedGoalPose.getRotation().getRadians())),
            Units.degreesToRadians(-maxRotationVelDegreePerSec.get()),
            Units.degreesToRadians(maxRotationVelDegreePerSec.get()));

    var fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translationVel.getX(), translationVel.getY(), rotationVel, currentPose.getRotation());

    swerve.setGoalVel(fieldRelativeSpeeds);
  }

  @Override
  public boolean isFinished() {
    return hasDone;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the swerve when command ends
    swerve.setGoalVel(new ChassisSpeeds());
  }

  private Pose2d getCurrentPose() {
    return RobotState.getOdometry().getEstimatedPose();
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

    var orgShiftXMeter = -shiftXT * maxLineupShiftingXMeter.get();
    var maybeHesitatedShiftXMeter =
        needHesitation
            ? Math.copySign(
                Math.max(Math.abs(orgShiftXMeter), minLineupHesitatedShiftingXMeter.get()),
                orgShiftXMeter)
            : orgShiftXMeter;

    var flippedShiftedGoalPose =
        flippedGoalPose.transformBy(
            GeomUtil.toTransform2d(
                maybeHesitatedShiftXMeter,
                Math.copySign(shiftYT * maxLineupShiftingYMeter.get() * 0.8, offset.getY())));

    return new Pose2d(flippedShiftedGoalPose.getTranslation(), goalPose.getRotation());
  }

  public boolean hasHeadingAtGoal() {
    return hasHeadingAtGoal;
  }

  public boolean hasDistanceWithin(double toleranceMeter) {
    return getCurrentPose().getTranslation().getDistance(goalPoseSupplier.get().getTranslation())
        <= toleranceMeter;
  }
}
