package frc.robot.subsystem.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.dashboard.Alert;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.Constants.DebugGroup;
import org.littletonrobotics.junction.Logger;

class Module {
  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/Module/DriveKp");
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/Module/DriveKd");
  private static final LoggedTunableNumber driveKs =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/Module/DriveKs");
  private static final LoggedTunableNumber steerKp =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/Module/SteerKp");
  private static final LoggedTunableNumber steerKd =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/Module/SteerKd");
  private static final LoggedTunableNumber steerKs =
      new LoggedTunableNumber(DebugGroup.SWERVE, "Swerve/Module/SteerKs");

  static {
    var driveSlot = SwerveConfig.getX2iDriveTalonConfig().Slot0;
    driveKp.initDefault(driveSlot.kP);
    driveKd.initDefault(driveSlot.kD);
    driveKs.initDefault(driveSlot.kS);

    var steerSlot = SwerveConfig.getX2iSteerTalonNoEncoderConfig().Slot0;
    steerKp.initDefault(steerSlot.kP);
    steerKd.initDefault(steerSlot.kD);
    steerKs.initDefault(steerSlot.kS);
  }

  private final String name;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  private final Alert driveMotorOfflineAlert;
  private final Alert steerMotorOfflineAlert;

  Module(ModuleIO io, String name) {
    this.io = io;
    this.name = "Module" + name;

    driveMotorOfflineAlert =
        new Alert(this.name + " drive motor offline!", Alert.AlertType.WARNING);
    steerMotorOfflineAlert =
        new Alert(this.name + " steer motor offline!", Alert.AlertType.WARNING);
  }

  void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve" + name, inputs);

    // Update gains when changed during live debugging
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setDrivePdf(driveKp.get(), driveKd.get(), driveKs.get()),
        driveKp,
        driveKd,
        driveKs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setSteerPdf(steerKp.get(), steerKd.get(), steerKs.get()),
        steerKp,
        steerKd,
        steerKs);

    // Display alerts
    driveMotorOfflineAlert.set(!inputs.driveMotorConnected);
    steerMotorOfflineAlert.set(!inputs.steerMotorConnected);
  }

  void setState(SwerveModuleState state) {
    io.setDriveVelocity(state.speedMetersPerSecond / SwerveConfig.WHEEL_RADIUS_METER);
    io.setSteerPosition(state.angle.getRadians());
  }

  void setState(SwerveModuleState state, SwerveModuleState ff) {
    io.setDriveVelocity(
        state.speedMetersPerSecond / SwerveConfig.WHEEL_RADIUS_METER,
        ff.speedMetersPerSecond / SwerveConfig.DRIVE_FF_KT);
    io.setSteerPosition(state.angle.getRadians());
  }

  SwerveModuleState getState() {
    return new SwerveModuleState(
        inputs.driveVelRadPerSec * SwerveConfig.WHEEL_RADIUS_METER, inputs.steerAbsPosition);
  }

  double getDrivePositionRad() {
    return inputs.drivePositionRad;
  }

  double getDriveVelRadPerSec() {
    return inputs.driveVelRadPerSec;
  }

  double getSteerPositionRad() {
    return inputs.steerAbsPosition.getRadians();
  }

  void setDriveCharacterizationCurrent(double currentAmp) {
    io.setDriveCurrent(currentAmp);
    io.setSteerPosition(0.0);
  }

  void stop() {
    io.stop();
  }
}
