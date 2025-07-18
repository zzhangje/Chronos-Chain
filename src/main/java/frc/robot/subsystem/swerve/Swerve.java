package frc.robot.subsystem.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.dashboard.Alert;
import frc.lib.interfaces.sensor.gyro.GyroIO;
import frc.lib.interfaces.sensor.gyro.GyroIOInputsAutoLogged;
import frc.lib.interfaces.sensor.gyro.GyroIOPigeon2;
import frc.robot.Constants.Ports;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;
import lombok.Setter;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Swerve {

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
  @Setter private Supplier<Double> customMaxTiltAccelScale = () -> 1.0;

  private final Alert gyroOfflineAlert = new Alert("Gyro offline!", Alert.AlertType.WARNING);

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
