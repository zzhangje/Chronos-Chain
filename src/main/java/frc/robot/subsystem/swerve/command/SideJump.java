package frc.robot.subsystem.swerve.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.lib.utils.AllianceFlipUtil;
import frc.reefscape.Field.Barge;
import frc.robot.Constants;
import frc.robot.Constants.DebugGroup;
import frc.robot.RobotState;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.SwerveConfig;
import java.util.Comparator;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public class SideJump extends Command {
  @RequiredArgsConstructor
  private enum Cage {
    LEFT(
        new LoggedTunableNumber(
            DebugGroup.SWERVE, "Swerve/CageAlignController/CageY/LeftYMeter", Barge.CAGE_LEFT_Y)),
    MID(
        new LoggedTunableNumber(
            DebugGroup.SWERVE, "Swerve/CageAlignController/CageY/MidYMeter", Barge.CAGE_MID_Y)),
    RIGHT(
        new LoggedTunableNumber(
            DebugGroup.SWERVE, "Swerve/CageAlignController/CageY/RightYMeter", Barge.CAGE_RIGHT_Y));

    private final LoggedTunableNumber ySupplier;

    public static Cage getTheClosest() {
      var cageList = List.of(LEFT, MID, RIGHT);
      var currentY = RobotState.getOdometry().getEstimatedPose().getY();

      var closest =
          cageList.stream()
              .min(Comparator.comparingDouble(cage -> Math.abs(currentY - cage.getY())));

      return closest.orElse(MID);
    }

    public double getY() {
      return AllianceFlipUtil.applyY(ySupplier.get());
    }

    public Cage getLeftSide() {
      return switch (this) {
        case LEFT, MID -> LEFT;
        case RIGHT -> MID;
      };
    }

    public Cage getRightSide() {
      return switch (this) {
        case RIGHT, MID -> RIGHT;
        case LEFT -> MID;
      };
    }
  }

  private static final LoggedTunableNumber translationKp =
      new LoggedTunableNumber(
          DebugGroup.SWERVE,
          "Swerve/CageAlignController/TranslationKp",
          Constants.MODE == Constants.Mode.REAL ? 1.9 : 5.0);
  private static final LoggedTunableNumber translationKd =
      new LoggedTunableNumber(
          DebugGroup.SWERVE,
          "Swerve/CageAlignController/TranslationKd",
          Constants.MODE == Constants.Mode.REAL ? 0.1 : 0.09);
  private static final LoggedTunableNumber maxTranslationVelMeterPerSec =
      new LoggedTunableNumber(
          DebugGroup.SWERVE, "Swerve/CageAlignController/MaxTranslationVelMeterPerSec", 3.5);
  private static final LoggedTunableNumber maxTranslationScalar =
      new LoggedTunableNumber(
          DebugGroup.SWERVE, "Swerve/CageAlignController/MaxTranslationScalar", 0.5);

  private static final LoggedTunableNumber rotationKp =
      new LoggedTunableNumber(
          DebugGroup.SWERVE,
          "Swerve/CageAlignController/RotationKp",
          Constants.MODE == Constants.Mode.REAL ? 4.0 : 4.5);
  private static final LoggedTunableNumber rotationKd =
      new LoggedTunableNumber(
          DebugGroup.SWERVE,
          "Swerve/CageAlignController/RotationKd",
          Constants.MODE == Constants.Mode.REAL ? 0.1 : 0.05);
  private static final LoggedTunableNumber rotationToleranceDegree =
      new LoggedTunableNumber(
          DebugGroup.SWERVE, "Swerve/CageAlignController/RotationToleranceDegree", 5.0);
  private static final LoggedTunableNumber maxRotationVelDegreePerSec =
      new LoggedTunableNumber(
          DebugGroup.SWERVE, "Swerve/CageAlignController/MaxRotationDegreePerSec", 360.0);

  private final PIDController yController;
  private final PIDController rotationController;
  private final Swerve swerve;
  private final DoubleSupplier xSupplier;
  private final BooleanSupplier wantLeftSupplier;
  private final BooleanSupplier wantRightSupplier;
  private Boolean lastWantLeft = false;
  private Boolean lastWantRight = false;
  private final BooleanSupplier wantDisableHeadingControllerSupplier;

  private boolean wantDisableHeadingController = false;
  private Cage cage;

  public SideJump(
      Swerve swerve,
      DoubleSupplier xSupplier,
      BooleanSupplier wantLeftSupplier,
      BooleanSupplier wantRightSupplier,
      BooleanSupplier wantDisableHeadingControllerSupplier) {
    this.swerve = swerve;
    this.xSupplier = xSupplier;
    this.wantLeftSupplier = wantLeftSupplier;
    this.wantRightSupplier = wantRightSupplier;
    this.wantDisableHeadingControllerSupplier = wantDisableHeadingControllerSupplier;

    yController = new PIDController(translationKp.get(), 0.0, translationKd.get());
    rotationController = new PIDController(rotationKp.get(), 0.0, rotationKd.get());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    cage = Cage.getTheClosest();

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    wantDisableHeadingController = wantDisableHeadingControllerSupplier.getAsBoolean();
    if (wantLeftSupplier.getAsBoolean() && !lastWantLeft) {
      cage = cage.getLeftSide();
    } else if (wantRightSupplier.getAsBoolean() && !lastWantRight) {
      cage = cage.getRightSide();
    }

    lastWantLeft = wantLeftSupplier.getAsBoolean();
    lastWantRight = wantRightSupplier.getAsBoolean();
    double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);

    var currentPose = RobotState.getOdometry().getEstimatedPose();
    var targetHeading = AllianceFlipUtil.apply(Barge.CLIMBING_HEADING);
    var targetY = cage.getY();

    var yFeedback = yController.calculate(currentPose.getY(), targetY);
    var rotationFeedback =
        rotationController.calculate(
            MathUtil.angleModulus(currentPose.getRotation().getRadians()),
            MathUtil.angleModulus(targetHeading.getRadians()));

    double yErrorMeter = targetY - currentPose.getY();
    Logger.recordOutput("Swerve/CageAlignController/YErrorMeter", yErrorMeter);

    var rotationErrorDegree = currentPose.getRotation().minus(targetHeading).getDegrees();
    Logger.recordOutput("Swerve/CageAlignController/RotationErrorDegree", rotationErrorDegree);

    var maxTranslationVelMeterPerSecVal = maxTranslationVelMeterPerSec.get();
    var maxRotationVelRadPerSec = Units.degreesToRadians(maxRotationVelDegreePerSec.get());

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x * SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC * maxTranslationScalar.get(),
            MathUtil.clamp(
                yFeedback, -maxTranslationVelMeterPerSecVal, maxTranslationVelMeterPerSecVal),
            wantDisableHeadingController
                ? 0.0
                : MathUtil.clamp(
                    rotationFeedback, -maxRotationVelRadPerSec, maxRotationVelRadPerSec),
            currentPose.getRotation());

    swerve.setGoalVel(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}
