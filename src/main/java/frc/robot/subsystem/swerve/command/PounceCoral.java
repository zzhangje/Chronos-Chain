package frc.robot.subsystem.swerve.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.lib.utils.GamePieceTracker;
import frc.robot.Constants.DebugGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystem.swerve.Swerve;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PounceCoral extends Command {
  private static final LoggedTunableNumber translationKp =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/PounceCoral/TranslationKp", 4.0);
  private static final LoggedTunableNumber translationKd =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/PounceCoral/TranslationKd", 1.0);
  private static final LoggedTunableNumber translationToleranceMeter =
      new LoggedTunableNumber(
          DebugGroup.SWERVE, "Swerve/PounceCoral/TranslationToleranceMeter", 0.08);
  private static final LoggedTunableNumber maxTranslationVelMeterPerSec =
      new LoggedTunableNumber(
          DebugGroup.SWERVE, "Swerve/PounceCoral/MaxTranslationVelMeterPerSec", 3.0);
  private static final LoggedTunableNumber minTranslationKeepingDistanceMeter =
      new LoggedTunableNumber(
          DebugGroup.SWERVE, "Swerve/PounceCoral/MinTranslationKeepingDistanceMeter", 0.2);

  private static final LoggedTunableNumber rotationKp =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/PounceCoral/RotationKp", 3.7);
  private static final LoggedTunableNumber rotationKd =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/PounceCoral/RotationKd", 0.3);
  private static final LoggedTunableNumber rotationToleranceDegree =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/PounceCoral/RotationToleranceDegree", 7.0);
  private static final LoggedTunableNumber maxRotationVelDegreePerSec =
      new LoggedTunableNumber(
          DebugGroup.SWERVE, "Swerve/PounceCoral/MaxRotationDegreePerSec", 360.0);

  private final PIDController translationController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController rotationController = new PIDController(0.0, 0.0, 0.0);
  private final GamePieceTracker gamePieceTracker;
  private final Supplier<Translation2d[]> searchdAreaVerticesSupplier;
  private final Swerve swerve;

  public PounceCoral(Supplier<Translation2d[]> searchdAreaVerticesSupplier, Swerve swerve) {
    this.gamePieceTracker = RobotContainer.getCoralTracker();
    this.searchdAreaVerticesSupplier = searchdAreaVerticesSupplier;
    this.swerve = swerve;

    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    translationController.reset();
    rotationController.reset();
  }

  @Override
  public void execute() {
    translationController.setP(translationKp.get());
    translationController.setD(translationKd.get());
    translationController.setTolerance(translationToleranceMeter.get());
    rotationController.setP(rotationKp.get());
    rotationController.setD(rotationKd.get());
    rotationController.setTolerance(rotationToleranceDegree.get());

    var goalCoralPose =
        gamePieceTracker.getBest(
            RobotContainer.getOdometry().getEstimatedPose(), searchdAreaVerticesSupplier.get());

    if (goalCoralPose.isEmpty()) {
      swerve.stop();
      return;
    }

    var goalCoralPosition = goalCoralPose.get().getTranslation();

    var currentPose = RobotContainer.getOdometry().getEstimatedPose();
    var effectiveDistance =
        currentPose.getTranslation().getDistance(goalCoralPosition)
            - minTranslationKeepingDistanceMeter.get();

    Logger.recordOutput("Swerve/PounceCoral/TranslationErrorMeter", effectiveDistance);

    var translationDir = goalCoralPosition.minus(currentPose.getTranslation()).getAngle();
    var translationFeedback = translationController.calculate(0.0, effectiveDistance);

    var rotationErrorDegree = currentPose.getRotation().minus(translationDir).getDegrees();

    var hasHeadingAtGoal = Math.abs(rotationErrorDegree) <= rotationToleranceDegree.get();

    var translationOutputScalar =
        hasHeadingAtGoal ? 1.0 : 1.0 - Math.abs(rotationErrorDegree) / 180.0;

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
                MathUtil.angleModulus(translationDir.getRadians())),
            Units.degreesToRadians(-maxRotationVelDegreePerSec.get()),
            Units.degreesToRadians(maxRotationVelDegreePerSec.get()));

    var chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translationVel.getX(), translationVel.getY(), rotationVel, currentPose.getRotation());

    swerve.setGoalVel(chassisSpeeds);
  }

  @Override
  public boolean isFinished() {
    var goalCoralPose =
        gamePieceTracker.getBest(
            RobotContainer.getOdometry().getEstimatedPose(), searchdAreaVerticesSupplier.get());
    if (goalCoralPose.isEmpty()) {
      return true;
    }

    var goalCoralPosition = goalCoralPose.get().getTranslation();
    var currentPose = RobotContainer.getOdometry().getEstimatedPose();
    var effectiveDistance =
        currentPose.getTranslation().getDistance(goalCoralPosition)
            - minTranslationKeepingDistanceMeter.get();

    return Math.abs(effectiveDistance) <= translationToleranceMeter.get();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}
