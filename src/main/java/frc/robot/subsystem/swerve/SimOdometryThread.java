package frc.robot.subsystem.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystem.swerve.Swerve.WheeledObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SimOdometryThread {
  private final Thread thread;
  private volatile boolean isRunning = false;
  private final List<Supplier<Double>> signals = new ArrayList<>(9);
  private final ArrayBlockingQueue<WheeledObservation> odometryCachedWheeledObservationQueue =
      new ArrayBlockingQueue<>(20);
  private final Timer timer = new Timer();

  SimOdometryThread(
      Supplier<Double> fl_drive_signal,
      Supplier<Double> fl_steer_signal,
      Supplier<Double> bl_drive_signal,
      Supplier<Double> bl_steer_signal,
      Supplier<Double> br_drive_signal,
      Supplier<Double> br_steer_signal,
      Supplier<Double> fr_drive_signal,
      Supplier<Double> fr_steer_signal) {
    thread = new Thread(this::run);
    thread.setName("PhoenixOdometryThread");
    thread.setDaemon(true);

    signals.add(0, fl_drive_signal);
    signals.add(1, fl_steer_signal);
    signals.add(2, bl_drive_signal);
    signals.add(3, bl_steer_signal);
    signals.add(4, br_drive_signal);
    signals.add(5, br_steer_signal);
    signals.add(6, fr_drive_signal);
    signals.add(7, fr_steer_signal);
  }

  ArrayBlockingQueue<WheeledObservation> start() {
    if (!isRunning) {
      thread.start();
      timer.start();
    }
    isRunning = true;

    return odometryCachedWheeledObservationQueue;
  }

  private void run() {
    while (isRunning) {
      if (timer.hasElapsed(Constants.LOOP_PERIOD_SEC)) {
        odometryCachedWheeledObservationQueue.offer(
            new WheeledObservation(
                Logger.getTimestamp() / 1.0e6,
                new SwerveModulePosition[] {
                  signalValue2SwerveModulePosition(signals.get(0).get(), signals.get(1).get()),
                  signalValue2SwerveModulePosition(signals.get(2).get(), signals.get(3).get()),
                  signalValue2SwerveModulePosition(signals.get(4).get(), signals.get(5).get()),
                  signalValue2SwerveModulePosition(signals.get(6).get(), signals.get(7).get()),
                },
                null));
        timer.reset();
      }
    }

    isRunning = false;
  }

  private SwerveModulePosition signalValue2SwerveModulePosition(
      double rawDrivePosition, double rawSteerPosition) {
    return new SwerveModulePosition(
        rawDrivePosition * SwerveConfig.WHEEL_RADIUS_METER,
        Rotation2d.fromRadians(rawSteerPosition));
  }
}
