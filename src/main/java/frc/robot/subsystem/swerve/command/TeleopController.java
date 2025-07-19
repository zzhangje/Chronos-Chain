package frc.robot.subsystem.swerve.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.lib.math.EqualsUtil;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.SwerveConfig;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TeleopController extends Command {
  private static final LoggedTunableNumber translationDeadband =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/TeleopController/TranslationDeadband", 0.12);
  private static final LoggedTunableNumber maxTranslationScalar =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/TeleopController/MaxTranslationScalar", 1.0);
  private static final LoggedTunableNumber rotationDeadband =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/TeleopController/RotationDeadband", 0.08);
  private static final LoggedTunableNumber usualRotationScalar =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/TeleopController/UsualRotationScalar", 0.8);
  private static final LoggedTunableNumber escapeRotationScalar =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/TeleopController/EscapeRotationScalar", 1.0);

  private static final LoggedTunableNumber headingMaintainerKp =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/TeleopController/HeadingMaintainer/Kp", 3.5);
  private static final LoggedTunableNumber headingMaintainerKd =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE, "Swerve/TeleopController/HeadingMaintainer/Kd", 0.0);
  private static final LoggedTunableNumber headingMaintainerToleranceDegree =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE,
          "Swerve/TeleopController/HeadingMaintainer/ToleranceDegree",
          3.0);
  private static final LoggedTunableNumber headingMaintainerMinOutputDegreePerSec =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE,
          "Swerve/TeleopController/HeadingMaintainer/MinOutputDegreePerSec",
          1.0);
  private static final LoggedTunableNumber headingMaintainerMaxOutputDegreePerSec =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE,
          "Swerve/TeleopController/HeadingMaintainer/MaxOutputDegreePerSec",
          90.0);
  private static final LoggedTunableNumber headingMaintainerMinEnableVelDegreePerSec =
      new LoggedTunableNumber(
          Constants.DebugGroup.SWERVE,
          "Swerve/TeleopController/HeadingMaintainer/MinEnableVelDegreePerSec",
          90.0);

  private final Swerve swerve;
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private final Supplier<Double> omegaSupplier;
  private final Supplier<Boolean> wantEscapeSupplier;

  private final PIDController headingMaintainer;
  private boolean enableHeadingMaintainer = false;

  public TeleopController(
      Swerve swerve,
      Supplier<Double> xSupplier,
      Supplier<Double> ySupplier,
      Supplier<Double> omegaSupplier,
      Supplier<Boolean> wantEscapeSupplier) {
    this.swerve = swerve;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.wantEscapeSupplier = wantEscapeSupplier;

    headingMaintainer =
        new PIDController(
            headingMaintainerKp.get(), 0.0, headingMaintainerKd.get(), Constants.LOOP_PERIOD_SEC);
    headingMaintainer.enableContinuousInput(-Math.PI, Math.PI);
    headingMaintainer.setTolerance(Units.degreesToRadians(headingMaintainerToleranceDegree.get()));

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    var translation = calcTranslation();
    var rotation = calcRotation();

    headingMaintainer.setPID(headingMaintainerKp.get(), 0.0, headingMaintainerKd.get());
    headingMaintainer.setTolerance(Units.degreesToRadians(headingMaintainerToleranceDegree.get()));

    var yawVelRadPerSec = RobotState.getOdometry().getRobotCentricVel().dtheta;
    var yawRad = RobotState.getOdometry().getEstimatedPose().getRotation().getRadians();

    boolean wantEscape = wantEscapeSupplier.get();
    if (!wantEscape
        && EqualsUtil.epsilonEquals(rotation, 0.0, 0.0)
        && EqualsUtil.epsilonEquals(
            yawVelRadPerSec,
            0.0,
            Units.degreesToRadians(headingMaintainerMinEnableVelDegreePerSec.get()))) {
      if (!enableHeadingMaintainer) {
        headingMaintainer.setSetpoint(yawRad);
        enableHeadingMaintainer = true;
      }
    } else {
      disableHeadingMaintainer();
    }

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      translation = translation.rotateBy(Rotation2d.fromRadians(Math.PI));
    }

    var headingMaintainerMaxOutputRadPerSec =
        Units.degreesToRadians(headingMaintainerMaxOutputDegreePerSec.get());

    var headingMaintainerOutput =
        enableHeadingMaintainer
            ? MathUtil.clamp(
                MathUtil.applyDeadband(
                    headingMaintainer.calculate(yawRad),
                    Units.degreesToRadians(headingMaintainerMinOutputDegreePerSec.get())),
                -headingMaintainerMaxOutputRadPerSec,
                headingMaintainerMaxOutputRadPerSec)
            : 0.0;
    Logger.recordOutput(
        "Swerve/TeleopController/HeadingMaintainer/Output", headingMaintainerOutput);

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX() * SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC,
            translation.getY() * SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC,
            rotation * SwerveConfig.MAX_ANGULAR_VEL_RAD_PER_SEC + headingMaintainerOutput,
            RobotState.getOdometry().getEstimatedPose().getRotation());

    swerve.setGoalVel(speeds);
  }

  private Translation2d calcTranslation() {
    double x = xSupplier.get();
    double y = ySupplier.get();
    var magnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), translationDeadband.get())
            * maxTranslationScalar.get();
    var direction = magnitude < 1e-16 ? new Rotation2d() : new Rotation2d(x, y);

    return new Translation2d(magnitude * magnitude, direction);
  }

  private double calcRotation() {
    double omega = omegaSupplier.get();
    boolean wantEscape = wantEscapeSupplier.get();
    var magnitude =
        MathUtil.applyDeadband(omega, rotationDeadband.get())
            * (wantEscape ? escapeRotationScalar.get() : usualRotationScalar.get());

    return Math.copySign(magnitude * magnitude, magnitude);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  public void disableHeadingMaintainer() {
    headingMaintainer.reset();
    enableHeadingMaintainer = false;
  }

  public void resetHeadingMaintainerSetpointToCurrent() {
    var yawRad = RobotState.getOdometry().getEstimatedPose().getRotation().getRadians();
    headingMaintainer.setSetpoint(yawRad);
  }
}
