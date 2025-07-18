package frc.robot.subsystem.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.subsystem.swerve.Swerve.WheeledObservation;
import java.util.concurrent.ArrayBlockingQueue;
import org.littletonrobotics.junction.Logger;

class PhoenixOdometryThread {
  private final Thread thread;
  private volatile boolean isRunning = false;
  private final BaseStatusSignal[] signals = new BaseStatusSignal[9];
  private final ArrayBlockingQueue<WheeledObservation> odometryCachedWheeledObservationQueue =
      new ArrayBlockingQueue<>(20);

  PhoenixOdometryThread(
      StatusSignal<Angle> fl_drive_signal,
      StatusSignal<Angle> fl_steer_signal,
      StatusSignal<Angle> bl_drive_signal,
      StatusSignal<Angle> bl_steer_signal,
      StatusSignal<Angle> br_drive_signal,
      StatusSignal<Angle> br_steer_signal,
      StatusSignal<Angle> fr_drive_signal,
      StatusSignal<Angle> fr_steer_signal,
      StatusSignal<Angle> yaw_signal) {
    thread = new Thread(this::run);
    thread.setName("PhoenixOdometryThread");
    thread.setDaemon(true);

    signals[0] = fl_drive_signal;
    signals[1] = fl_steer_signal;
    signals[2] = bl_drive_signal;
    signals[3] = bl_steer_signal;
    signals[4] = br_drive_signal;
    signals[5] = br_steer_signal;
    signals[6] = fr_drive_signal;
    signals[7] = fr_steer_signal;
    signals[8] = yaw_signal;
  }

  ArrayBlockingQueue<WheeledObservation> start() {
    if (!isRunning) {
      thread.start();
    }
    isRunning = true;

    return odometryCachedWheeledObservationQueue;
  }

  private void run() {
    BaseStatusSignal.setUpdateFrequencyForAll(SwerveConfig.ODOMETRY_FREQUENCY_HZ, signals);

    while (isRunning) {
      BaseStatusSignal.waitForAll(Constants.LOOP_PERIOD_SEC, signals);

      odometryCachedWheeledObservationQueue.offer(
          new WheeledObservation(
              Logger.getTimestamp() / 1.0e6,
              new SwerveModulePosition[] {
                signalValue2SwerveModulePosition(
                    signals[0].getValueAsDouble(), signals[1].getValueAsDouble()),
                signalValue2SwerveModulePosition(
                    signals[2].getValueAsDouble(), signals[3].getValueAsDouble()),
                signalValue2SwerveModulePosition(
                    signals[4].getValueAsDouble(), signals[5].getValueAsDouble()),
                signalValue2SwerveModulePosition(
                    signals[6].getValueAsDouble(), signals[7].getValueAsDouble()),
              },
              Rotation2d.fromDegrees(signals[8].getValueAsDouble())));
    }

    isRunning = false;
  }

  private SwerveModulePosition signalValue2SwerveModulePosition(
      double rawDrivePosition, double rawSteerPosition) {
    return new SwerveModulePosition(
        Units.rotationsToRadians(rawDrivePosition) * SwerveConfig.WHEEL_RADIUS_METER,
        Rotation2d.fromRotations(rawSteerPosition));
  }
}
