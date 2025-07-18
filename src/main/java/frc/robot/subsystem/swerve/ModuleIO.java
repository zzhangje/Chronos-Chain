package frc.robot.subsystem.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    boolean driveMotorConnected = false;
    boolean steerMotorConnected = false;

    double driveVelRadPerSec = 0.0;
    double drivePositionRad = 0.0;
    double driveOutputVoltageVolt = 0.0;
    double driveSupplyCurrentAmp = 0.0;
    double driveStatorCurrentAmp = 0.0;

    double steerVelRadPerSec = 0.0;
    Rotation2d steerAbsPosition = new Rotation2d();
    double steerOutputVoltageVolt = 0.0;
    double steerSupplyCurrentAmp = 0.0;
    double steerStatorCurrentAmp = 0.0;
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void setDrivePdf(double kp, double kd, double ks) {}

  default void setSteerPdf(double kp, double kd, double ks) {}

  default void setDriveVelocity(double velRadPerSec) {
    setDriveVelocity(velRadPerSec, 0.0);
  }

  default void setDriveVelocity(double velRadPerSec, double torqueAmp) {}

  default void setDriveCurrent(double currentAmp) {}

  default void setSteerPosition(double angleRad) {}

  default void stop() {}
}
