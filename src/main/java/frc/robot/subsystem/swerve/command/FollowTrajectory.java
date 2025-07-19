package frc.robot.subsystem.swerve.command;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.lib.math.GeomUtil;
import frc.robot.Constants.DebugGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystem.swerve.Swerve;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class FollowTrajectory extends Command {
  private static final LoggedTunableNumber translationKp =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/FollowTrajectory/TranslationKp", 8.0);
  private static final LoggedTunableNumber translationKd =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/FollowTrajectory/TranslationKd", 0.0);
  private static final LoggedTunableNumber rotationKp =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/FollowTrajectory/RotationKp", 5.0);
  private static final LoggedTunableNumber rotationKd =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/FollowTrajectory/RotationKd", 0.0);

  private final Swerve swerve;
  private final Supplier<Trajectory<SwerveSample>> trajectorySupplier;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;
  private final Timer timer = new Timer();

  @Getter
  private List<Vector<N2>> moduleForces =
      IntStream.range(0, 4).boxed().map(i -> VecBuilder.fill(0, 0)).toList();

  private SwerveSample setpoint = null;

  public FollowTrajectory(Swerve swerve, Supplier<Trajectory<SwerveSample>> trajectorySupplier) {
    this.swerve = swerve;
    this.trajectorySupplier = trajectorySupplier;

    xController = new PIDController(translationKp.get(), 0, translationKd.get());
    yController = new PIDController(translationKp.get(), 0, translationKd.get());
    rotationController = new PIDController(rotationKp.get(), 0, rotationKd.get());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
    Logger.recordOutput(
        "Swerve/FollowTrajectory/TrajectoryPoses", trajectorySupplier.get().getPoses());
  }

  @Override
  public void initialize() {
    setpoint = trajectorySupplier.get().getInitialSample(false).get();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (!isFinished()) {
      setpoint = trajectorySupplier.get().sampleAt(timer.get(), false).get();
    }
    Logger.recordOutput("Swerve/FollowTrajectory/setpointPose", setpoint.getPose());
    Logger.recordOutput("Swerve/FollowTrajectory/setpointVel/Vx", setpoint.vx);
    Logger.recordOutput("Swerve/FollowTrajectory/setpointVel/Vy", setpoint.vy);
    Logger.recordOutput("Swerve/FollowTrajectory/setpointVel/Rotation", setpoint.omega);

    var trajectoryVel =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            setpoint.vx, setpoint.vy, setpoint.omega, setpoint.getPose().getRotation());

    RobotContainer.getOdometry().addTrajectoryVel(trajectoryVel.toTwist2d());

    var currentPose = RobotContainer.getOdometry().getEstimatedPose();
    var setpointPose = setpoint.getPose();

    var xFeedback = xController.calculate(currentPose.getX(), setpointPose.getX());
    var yFeedback = yController.calculate(currentPose.getY(), setpointPose.getY());
    var rotationFeedback =
        rotationController.calculate(
            MathUtil.angleModulus(currentPose.getRotation().getRadians()),
            MathUtil.angleModulus(setpointPose.getRotation().getRadians()));

    var setpointHeading = setpointPose.getRotation();
    var moduleForcesX = setpoint.moduleForcesX();
    var moduleForcesY = setpoint.moduleForcesY();

    // [FL, FR, BL, BR] -> [FL, BL, BR, FR]
    moduleForces = new ArrayList<>(4);
    moduleForces.add(fixModuleForce(moduleForcesX[0], moduleForcesY[0], setpointHeading));
    moduleForces.add(fixModuleForce(moduleForcesX[2], moduleForcesY[2], setpointHeading));
    moduleForces.add(fixModuleForce(moduleForcesX[3], moduleForcesY[3], setpointHeading));
    moduleForces.add(fixModuleForce(moduleForcesX[1], moduleForcesY[1], setpointHeading));

    Logger.recordOutput(
        "Swerve/FollowTrajectory/TranslationError",
        currentPose.getTranslation().getDistance(setpointPose.getTranslation()));
    Logger.recordOutput(
        "Swerve/FollowTrajectory/RotationError",
        currentPose.getRotation().minus(setpointPose.getRotation()));

    ChassisSpeeds targetSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            setpoint.vx + xFeedback,
            setpoint.vy + yFeedback,
            setpoint.omega + rotationFeedback,
            currentPose.getRotation());

    swerve.setGoalVelFF(targetSpeeds, moduleForces);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectorySupplier.get().getTotalTime());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
    Logger.recordOutput("Swerve/FollowTrajectory/Completed", !interrupted);
    Logger.recordOutput("Swerve/FollowTrajectory/TrajectoryPoses", new Pose2d[0]);
  }

  public Pose2d getInitialPose() {
    return trajectorySupplier.get().getInitialSample(false).get().getPose();
  }

  private static Vector<N2> fixModuleForce(double x, double y, Rotation2d headingInField) {
    return new Translation2d(x, y)
        .rotateBy(Rotation2d.fromRadians(headingInField.getRadians()).unaryMinus())
        .toVector();
  }
}
