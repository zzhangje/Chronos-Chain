package frc.robot.subsystem.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.CanId;
import frc.lib.utils.Phoenix6Helper;
import lombok.Getter;

class ModuleIOKrakenFOC implements ModuleIO {
  private final TalonFXConfiguration driveTalonConfig;
  private final TalonFXConfiguration steerTalonConfig;

  // Hardware
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder cancoder;

  // Signal
  private final StatusSignal<AngularVelocity> driveVel;
  @Getter private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<Voltage> driveOutputVoltage;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Current> driveStatorCurrent;

  private final StatusSignal<AngularVelocity> steerVel;
  @Getter private final StatusSignal<Angle> steerAbsPosition;
  private final StatusSignal<Voltage> steerOutputVoltage;
  private final StatusSignal<Current> steerSupplyCurrent;
  private final StatusSignal<Current> steerStatorCurrent;

  // Control request
  private final VelocityTorqueCurrentFOC driveVelSetter = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC driveCurrentSetter = new TorqueCurrentFOC(0.0);
  private final NeutralOut driveNeutralSetter = new NeutralOut();
  private final PositionTorqueCurrentFOC steerPositionSetter = new PositionTorqueCurrentFOC(0.0);
  private final NeutralOut steerNeutralSetter = new NeutralOut();

  public ModuleIOKrakenFOC(
      String name,
      CanId driveId,
      CanId steerId,
      CanId cancoderId,
      SwerveConfig.ModuleConfig config) {
    var wrappedName = "[" + name + "]";
    driveMotor = new TalonFX(driveId.id(), driveId.bus());
    steerMotor = new TalonFX(steerId.id(), steerId.bus());
    cancoder = new CANcoder(cancoderId.id(), cancoderId.bus());

    // Zero drive motor position
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " zero drive position", () -> driveMotor.setPosition(0));

    // Config drive motor
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " clear drive sticky fault", driveMotor::clearStickyFaults);
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " config drive",
        () -> driveMotor.getConfigurator().apply(config.driveTalonConfig()));

    driveTalonConfig = config.driveTalonConfig();

    // Config steer motor
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " clear steer sticky fault", steerMotor::clearStickyFaults);
    var realSteerTalonConfig = config.steerTalonConfig();
    realSteerTalonConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    realSteerTalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " config steer",
        () -> steerMotor.getConfigurator().apply(realSteerTalonConfig));

    steerTalonConfig = realSteerTalonConfig;

    // Config cancoder
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " config cancoder",
        () -> cancoder.getConfigurator().apply(config.cancoderConfig()));

    // Config Signal
    driveVel = driveMotor.getVelocity();
    drivePosition = driveMotor.getPosition();
    driveOutputVoltage = driveMotor.getMotorVoltage();
    driveSupplyCurrent = driveMotor.getSupplyCurrent();
    driveStatorCurrent = driveMotor.getStatorCurrent();

    steerVel = steerMotor.getVelocity();
    steerAbsPosition = steerMotor.getPosition();
    steerOutputVoltage = steerMotor.getMotorVoltage();
    steerSupplyCurrent = steerMotor.getSupplyCurrent();
    steerStatorCurrent = steerMotor.getStatorCurrent();

    // drivePosition and steerAbsPosition will be config by odometry thread
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " set signals update frequency",
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                driveVel,
                // drivePosition,
                driveOutputVoltage,
                driveSupplyCurrent,
                driveStatorCurrent,
                steerVel,
                // steerAbsPosition,
                steerOutputVoltage,
                steerSupplyCurrent,
                steerStatorCurrent));

    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " optimize drive CAN utilization", driveMotor::optimizeBusUtilization);
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " optimize steer CAN utilization", steerMotor::optimizeBusUtilization);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(
                driveVel, drivePosition, driveOutputVoltage, driveSupplyCurrent, driveStatorCurrent)
            .isOK();
    inputs.steerMotorConnected =
        BaseStatusSignal.refreshAll(
                steerVel,
                steerAbsPosition,
                steerOutputVoltage,
                steerSupplyCurrent,
                steerStatorCurrent)
            .isOK();

    inputs.driveVelRadPerSec = Units.rotationsToRadians(driveVel.getValueAsDouble());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveOutputVoltageVolt = driveOutputVoltage.getValueAsDouble();
    inputs.driveSupplyCurrentAmp = driveSupplyCurrent.getValueAsDouble();
    inputs.driveStatorCurrentAmp = driveStatorCurrent.getValueAsDouble();

    inputs.steerVelRadPerSec = Units.rotationsToRadians(steerVel.getValueAsDouble());
    inputs.steerAbsPosition = Rotation2d.fromRotations(steerAbsPosition.getValueAsDouble());
    inputs.steerOutputVoltageVolt = steerOutputVoltage.getValueAsDouble();
    inputs.steerSupplyCurrentAmp = steerSupplyCurrent.getValueAsDouble();
    inputs.steerStatorCurrentAmp = steerStatorCurrent.getValueAsDouble();
  }

  @Override
  public void setDriveCurrent(double currentAmp) {
    driveMotor.setControl(driveCurrentSetter.withOutput(currentAmp));
  }

  @Override
  public void setDrivePdf(double kp, double kd, double ks) {
    driveTalonConfig.Slot0.kP = kp;
    driveTalonConfig.Slot0.kD = kd;
    driveTalonConfig.Slot0.kS = ks;
    driveMotor.getConfigurator().apply(driveTalonConfig);
  }

  @Override
  public void setSteerPdf(double kp, double kd, double ks) {
    steerTalonConfig.Slot0.kP = kp;
    steerTalonConfig.Slot0.kD = kd;
    steerTalonConfig.Slot0.kS = ks;
    steerMotor.getConfigurator().apply(steerTalonConfig);
  }

  @Override
  public void setDriveVelocity(double velRadPerSec, double torqueAmp) {
    driveMotor.setControl(
        driveVelSetter
            .withVelocity(Units.radiansToRotations(velRadPerSec))
            .withFeedForward(torqueAmp));
  }

  @Override
  public void setSteerPosition(double angleRad) {
    steerMotor.setControl(steerPositionSetter.withPosition(Units.radiansToRotations(angleRad)));
  }

  @Override
  public void stop() {
    driveMotor.setControl(driveNeutralSetter);
    steerMotor.setControl(steerNeutralSetter);
  }
}
