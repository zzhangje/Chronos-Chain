package frc.robot.subsystem.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.Alert;
import frc.lib.interfaces.sensor.gyro.GyroIO;
import frc.lib.interfaces.sensor.gyro.GyroIOInputsAutoLogged;
import frc.lib.interfaces.sensor.gyro.GyroIOPigeon2;
import frc.lib.math.EqualsUtil;
import frc.lib.math.EqualsUtil.GeomExtensions;
import frc.lib.math.GeomUtil;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@ExtensionMethod({GeomUtil.class, GeomExtensions.class})
public class Swerve extends SubsystemBase {
  private final Module[] modules = new Module[4];
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveOdometryInputs swerveOdometryInputs = new SwerveOdometryInputs();
  private final ArrayBlockingQueue<WheeledObservation> odometryCachedWheeledObservationQueue;

  private SwerveModuleState[] lastGoalModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
      };

  private final Alert gyroOfflineAlert = new Alert("Gyro offline!", Alert.AlertType.WARNING);

  @Setter private Supplier<Double> customMaxTiltAccelScale = () -> 1.0;
  private List<Vector<N2>> moduleForces =
      IntStream.range(0, 4).boxed().map(i -> VecBuilder.fill(0, 0)).toList();
  private ChassisSpeeds goalVel = new ChassisSpeeds();
  private Boolean applyAccelLimitation = true;

  @Override
  public void periodic() {
    updateInputs();

    var currentVel = getVel();
    updateOdometry(currentVel);

    // Desaturate
    var rawGoalModuleStates = SwerveConfig.SWERVE_KINEMATICS.toSwerveModuleStates(goalVel);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        rawGoalModuleStates, SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC);
    goalVel = SwerveConfig.SWERVE_KINEMATICS.toChassisSpeeds(rawGoalModuleStates);

    // 1690 Orbit accel limitation
    if (applyAccelLimitation) {
      goalVel = clipAcceleration(currentVel, goalVel);
    }

    // Dynamics compensation
    goalVel = ChassisSpeeds.discretize(goalVel, Constants.LOOP_PERIOD_SEC);

    // Use last goal angle for module if chassis want stop completely
    var goalModuleStates = SwerveConfig.SWERVE_KINEMATICS.toSwerveModuleStates(goalVel);
    if (goalVel.toTwist2d().epsilonEquals(new Twist2d())) {
      for (int i = 0; i < modules.length; i++) {
        goalModuleStates[i].angle = lastGoalModuleStates[i].angle;
        goalModuleStates[i].speedMetersPerSecond = 0.0;
      }
    }

    var optimizedGoalModuleStates = new SwerveModuleState[4];
    var optimizedGoalModuleTorques = new SwerveModuleState[4];

    for (int i = 0; i < modules.length; i++) {
      // Optimize setpoints
      optimizedGoalModuleStates[i] = goalModuleStates[i];
      optimizedGoalModuleStates[i].optimize(modules[i].getState().angle);

      var wheelDirection =
          VecBuilder.fill(
              optimizedGoalModuleStates[i].angle.getCos(),
              optimizedGoalModuleStates[i].angle.getSin());
      var wheelForces = moduleForces.get(i);
      var wheelTorque = wheelForces.dot(wheelDirection) * SwerveConfig.WHEEL_RADIUS_METER;
      optimizedGoalModuleTorques[i] =
          new SwerveModuleState(wheelTorque, optimizedGoalModuleStates[i].angle);

      modules[i].setState(optimizedGoalModuleStates[i], optimizedGoalModuleTorques[i]);
    }

    lastGoalModuleStates = goalModuleStates;

    Logger.recordOutput("Swerve/SwerveStates/GoalModuleStates", goalModuleStates);
    Logger.recordOutput("Swerve/FinalGoalVel", goalVel);
    Logger.recordOutput("Swerve/SwerveStates/OptimizedGoalModuleStates", optimizedGoalModuleStates);
    Logger.recordOutput(
        "Swerve/SwerveStates/OptimizedGoalModuleTorques", optimizedGoalModuleTorques);
  }

  private ChassisSpeeds clipAcceleration(
      final ChassisSpeeds currentVel, final ChassisSpeeds goalVel) {
    var currentTranslationVel =
        new Translation2d(currentVel.vxMetersPerSecond, currentVel.vyMetersPerSecond);

    var goalTranslationVel =
        new Translation2d(goalVel.vxMetersPerSecond, goalVel.vyMetersPerSecond);

    var rawAccelPerLoop =
        goalTranslationVel.minus(currentTranslationVel).div(Constants.LOOP_PERIOD_SEC);

    var customMaxTiltAccelScaleVal = customMaxTiltAccelScale.get();
    Logger.recordOutput("Swerve/customMaxTiltAccelScale", customMaxTiltAccelScaleVal);
    var maxTiltAccelXPerLoop =
        SwerveConfig.maxTiltAccelXMeterPerSecPerLoop.get() * customMaxTiltAccelScaleVal;
    var maxTiltAccelYPerLoop =
        SwerveConfig.maxTiltAccelYMeterPerSecPerLoop.get() * customMaxTiltAccelScaleVal;

    var tiltLimitedAccelPerLoop =
        new Translation2d(
            MathUtil.clamp(rawAccelPerLoop.getX(), -maxTiltAccelXPerLoop, maxTiltAccelXPerLoop),
            MathUtil.clamp(rawAccelPerLoop.getY(), -maxTiltAccelYPerLoop, maxTiltAccelYPerLoop));

    var skidLimitedAccelPerLoop = new Translation2d();

    if (!EqualsUtil.epsilonEquals(tiltLimitedAccelPerLoop.getNorm(), 0.0)) {
      skidLimitedAccelPerLoop =
          new Translation2d(
              MathUtil.clamp(
                  tiltLimitedAccelPerLoop.getNorm(),
                  -SwerveConfig.maxSkidAccelMeterPerSecPerLoop.get(),
                  SwerveConfig.maxSkidAccelMeterPerSecPerLoop.get()),
              tiltLimitedAccelPerLoop.toRotation2d());
    }

    var calculatedDeltaVel = skidLimitedAccelPerLoop.times(Constants.LOOP_PERIOD_SEC);

    var limitedGoalVelTranslation = currentTranslationVel.plus(calculatedDeltaVel);

    return new ChassisSpeeds(
        limitedGoalVelTranslation.getX(),
        limitedGoalVelTranslation.getY(),
        goalVel.omegaRadiansPerSecond);
  }

  @AutoLogOutput(key = "Swerve/RobotCentricVel")
  public ChassisSpeeds getVel() {
    var vel = SwerveConfig.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    if (gyroInputs.connected) {
      vel.omegaRadiansPerSecond = gyroInputs.yawVelocityRadPerSec;
    }

    return vel;
  }

  @AutoLogOutput(key = "Swerve/ModuleStates")
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
  }

  private void updateInputs() {
    gyroIO.updateInputs(gyroInputs);
    gyroOfflineAlert.set(!gyroInputs.connected);
    Logger.processInputs("Swerve/Gyro", gyroInputs);

    for (var module : modules) {
      module.updateInputs();
    }

    var odometrySampleArray =
        odometryCachedWheeledObservationQueue.toArray(WheeledObservation[]::new);
    odometryCachedWheeledObservationQueue.clear();

    var sampleNum = odometrySampleArray.length;
    swerveOdometryInputs.odometryTimestamps = new double[sampleNum];
    swerveOdometryInputs.odometryFLPositions = new SwerveModulePosition[sampleNum];
    swerveOdometryInputs.odometryBLPositions = new SwerveModulePosition[sampleNum];
    swerveOdometryInputs.odometryBRPositions = new SwerveModulePosition[sampleNum];
    swerveOdometryInputs.odometryFRPositions = new SwerveModulePosition[sampleNum];
    swerveOdometryInputs.odometryYaws = new Rotation2d[sampleNum];

    for (int i = 0; i < sampleNum; i++) {
      swerveOdometryInputs.odometryTimestamps[i] = odometrySampleArray[i].timestamp();
      swerveOdometryInputs.odometryFLPositions[i] = odometrySampleArray[i].wheelPositions()[0];
      swerveOdometryInputs.odometryBLPositions[i] = odometrySampleArray[i].wheelPositions()[1];
      swerveOdometryInputs.odometryBRPositions[i] = odometrySampleArray[i].wheelPositions()[2];
      swerveOdometryInputs.odometryFRPositions[i] = odometrySampleArray[i].wheelPositions()[3];
      swerveOdometryInputs.odometryYaws[i] = odometrySampleArray[i].yaw();
    }

    Logger.processInputs("Swerve/Odometry", swerveOdometryInputs);
  }

  private void updateOdometry(final ChassisSpeeds currentVel) {
    // TODO:
  }

  private Swerve(
      ModuleIO flModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      ModuleIO frModuleIO,
      GyroIO gyroIO,
      ArrayBlockingQueue<WheeledObservation> odometryCachedWheeledObservationQueue) {

    this.gyroIO = gyroIO;
    this.odometryCachedWheeledObservationQueue = odometryCachedWheeledObservationQueue;

    modules[0] = new Module(flModuleIO, SwerveConfig.FL_MODULE_NAME);
    modules[1] = new Module(blModuleIO, SwerveConfig.BL_MODULE_NAME);
    modules[2] = new Module(brModuleIO, SwerveConfig.BR_MODULE_NAME);
    modules[3] = new Module(frModuleIO, SwerveConfig.FR_MODULE_NAME);
  }

  public Swerve createReal() {
    var flModuleIo =
        new ModuleIOKrakenFOC(
            SwerveConfig.FL_MODULE_NAME,
            Ports.Can.FL_DRIVE_MOTOR,
            Ports.Can.FL_STEER_MOTOR,
            Ports.Can.FL_STEER_SENSOR,
            SwerveConfig.FL_MODULE_CONFIG);

    var blModuleIo =
        new ModuleIOKrakenFOC(
            SwerveConfig.BL_MODULE_NAME,
            Ports.Can.BL_DRIVE_MOTOR,
            Ports.Can.BL_STEER_MOTOR,
            Ports.Can.BL_STEER_SENSOR,
            SwerveConfig.BL_MODULE_CONFIG);

    var brModuleIo =
        new ModuleIOKrakenFOC(
            SwerveConfig.BR_MODULE_NAME,
            Ports.Can.BR_DRIVE_MOTOR,
            Ports.Can.BR_STEER_MOTOR,
            Ports.Can.BR_STEER_SENSOR,
            SwerveConfig.BR_MODULE_CONFIG);

    var frModuleIo =
        new ModuleIOKrakenFOC(
            SwerveConfig.FR_MODULE_NAME,
            Ports.Can.FR_DRIVE_MOTOR,
            Ports.Can.FR_STEER_MOTOR,
            Ports.Can.FR_STEER_SENSOR,
            SwerveConfig.FR_MODULE_CONFIG);

    var gyroIOPigeon2 = new GyroIOPigeon2(Ports.Can.CHASSIS_PIGEON);

    var odometryCachedWheeledObservationQueue =
        new PhoenixOdometryThread(
                flModuleIo.getDrivePosition(),
                flModuleIo.getSteerAbsPosition(),
                blModuleIo.getDrivePosition(),
                blModuleIo.getSteerAbsPosition(),
                brModuleIo.getDrivePosition(),
                brModuleIo.getSteerAbsPosition(),
                frModuleIo.getDrivePosition(),
                frModuleIo.getSteerAbsPosition(),
                gyroIOPigeon2.getYaw())
            .start();

    return new Swerve(
        flModuleIo,
        blModuleIo,
        brModuleIo,
        frModuleIo,
        gyroIOPigeon2,
        odometryCachedWheeledObservationQueue);
  }

  public Swerve createSim() {
    var flModuleIo = new ModuleIOSim();
    var blModuleIo = new ModuleIOSim();
    var brModuleIo = new ModuleIOSim();
    var frModuleIo = new ModuleIOSim();
    var gyroIOSim = new GyroIO() {};
    var odometryCachedWheeledObservationQueue =
        new SimOdometryThread(
                flModuleIo::getDrivePositionRad,
                flModuleIo::getSteerPositionRad,
                blModuleIo::getDrivePositionRad,
                blModuleIo::getSteerPositionRad,
                brModuleIo::getDrivePositionRad,
                brModuleIo::getSteerPositionRad,
                frModuleIo::getDrivePositionRad,
                frModuleIo::getSteerPositionRad)
            .start();

    return new Swerve(
        flModuleIo,
        blModuleIo,
        brModuleIo,
        frModuleIo,
        gyroIOSim,
        odometryCachedWheeledObservationQueue);
  }

  public Swerve createIO() {
    return new Swerve(
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new GyroIO() {},
        new ArrayBlockingQueue<>(20));
  }

  public record WheeledObservation(
      double timestamp, SwerveModulePosition[] wheelPositions, Rotation2d yaw) {}

  static class SwerveOdometryInputs implements LoggableInputs {
    double[] odometryTimestamps;
    SwerveModulePosition[] odometryFLPositions;
    SwerveModulePosition[] odometryBLPositions;
    SwerveModulePosition[] odometryBRPositions;
    SwerveModulePosition[] odometryFRPositions;
    Rotation2d[] odometryYaws;

    @Override
    public void toLog(LogTable table) {
      table.put("OdometryTimestamps", odometryTimestamps);
      table.put("OdometryFLPositions", odometryFLPositions);
      table.put("OdometryBLPositions", odometryBLPositions);
      table.put("OdometryBRPositions", odometryBRPositions);
      table.put("OdometryFRPositions", odometryFRPositions);

      if (odometryYaws.length != 0 && odometryYaws[0] != null) {
        table.put("OdometryYaws", odometryYaws);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
      odometryFLPositions = table.get("OdometryFLPositions", odometryFLPositions);
      odometryBLPositions = table.get("OdometryBLPositions", odometryBLPositions);
      odometryBRPositions = table.get("OdometryBRPositions", odometryBRPositions);
      odometryFRPositions = table.get("OdometryFRPositions", odometryFRPositions);
      odometryYaws = table.get("OdometryYaws", odometryYaws);
    }
  }
}
