package frc.lib.interfaces.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.interfaces.CanId;
import frc.lib.utils.Phoenix6Helper;
import java.util.ArrayList;
import java.util.List;

public class GenericArmIOKrakenCancoder implements GenericArmIO {
  private final String name;

  private final TalonFX master;
  private final CANcoder cancoder;
  private final List<TalonFX> slaves = new ArrayList<>();

  private final TalonFXConfiguration masterConfig;

  private final StatusSignal<AngularVelocity> vel;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Voltage> outputVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;

  private final PositionTorqueCurrentFOC positionSetter;
  private final DynamicMotionMagicTorqueCurrentFOC motionMagicSetter;
  private final TorqueCurrentFOC currentSetter = new TorqueCurrentFOC(0.0);
  private final VoltageOut voltageSetter = new VoltageOut(0.0);
  private final NeutralOut neutralSetter = new NeutralOut();
  private final List<Follower> slaveSetters = new ArrayList<>();

  public GenericArmIOKrakenCancoder(
      String name,
      CanId masterId,
      TalonFXConfiguration masterTalonConfig,
      CanId cancoderId,
      boolean isCancoderCcw,
      double cancoderOffset,
      double absoluteSensorDiscontinuityPoint) {
    this.name = name;
    master = new TalonFX(masterId.id(), masterId.bus());
    cancoder = new CANcoder(cancoderId.id(), cancoderId.bus());

    masterConfig = masterTalonConfig;
    masterConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    masterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    var wrappedName = "[" + name + "]";

    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " clear sticky fault", master::clearStickyFaults);
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " config", () -> master.getConfigurator().apply(masterConfig));

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = cancoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        isCancoderCcw
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = absoluteSensorDiscontinuityPoint;
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " config cancoder", () -> cancoder.getConfigurator().apply(cancoderConfig));

    positionSetter = new PositionTorqueCurrentFOC(0.0);
    motionMagicSetter =
        new DynamicMotionMagicTorqueCurrentFOC(Units.degreesToRotations(0.0), 0.0, 0.0, 0.0);

    vel = master.getVelocity();
    position = master.getPosition();
    outputVoltage = master.getMotorVoltage();
    supplyCurrent = master.getSupplyCurrent();
    statorCurrent = master.getStatorCurrent();

    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " set signals update frequency",
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100.0, vel, position, outputVoltage, supplyCurrent, statorCurrent));

    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " optimize CAN utilization", master::optimizeBusUtilization);
  }

  public GenericArmIOKrakenCancoder withFollower(CanId id, boolean isInvert) {
    var slave = new TalonFX(id.id(), id.bus());

    var slaveConfig = new TalonFXConfiguration();
    slaveConfig.MotorOutput.NeutralMode = masterConfig.MotorOutput.NeutralMode;

    var slaveSetter = new Follower(master.getDeviceID(), isInvert);

    var wrappedName = "[" + name + "Slave" + slaves.size() + "]";
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " clear sticky fault", slave::clearStickyFaults);
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " config", () -> slave.getConfigurator().apply(slaveConfig));
    Phoenix6Helper.checkErrorAndRetry(wrappedName + " follow", () -> slave.setControl(slaveSetter));
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " optimize CAN utilization", slave::optimizeBusUtilization);

    slaves.add(slave);
    slaveSetters.add(slaveSetter);

    return this;
  }

  @Override
  public void updateInputs(GenericArmIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(vel, position, outputVoltage, supplyCurrent, statorCurrent)
            .isOK();

    inputs.velRadPerSec = Units.rotationsToRadians(vel.getValueAsDouble());
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.outputVoltageVolt = outputVoltage.getValueAsDouble();
    inputs.supplyCurrentAmp = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmp = statorCurrent.getValueAsDouble();
  }

  @Override
  public void setPosition(double positionRad, double feedforward) {
    master.setControl(
        positionSetter
            .withPosition(Units.radiansToRotations(positionRad))
            .withFeedForward(feedforward));
    setSlavesFollow();
  }

  @Override
  public void setPosition(
      double positionRad, double velRadPerSec, double accelRadPerSec2, double feedforward) {

    master.setControl(
        positionSetter
            .withPosition(Units.radiansToRotations(positionRad))
            .withFeedForward(feedforward));
    setSlavesFollow();
    // FIXME: TESTING
    // master.setControl(
    //     motionMagicSetter
    //         .withPosition(Units.radiansToRotations(positionRad))
    //         .withFeedForward(feedforward)
    //         .withVelocity(Units.radiansToRotations(velRadPerSec))
    //         .withAcceleration(Units.radiansToRotations(accelRadPerSec2)));
    // setSlavesFollow();
  }

  @Override
  public void setPdf(double kp, double kd, double ks, double kg) {
    masterConfig.Slot0.kP = kp;
    masterConfig.Slot0.kD = kd;
    masterConfig.Slot0.kS = ks;
    masterConfig.Slot0.kG = kg;
    master.getConfigurator().apply(masterConfig);
  }

  @Override
  public void setCurrent(double currentAmp) {
    master.setControl(currentSetter.withOutput(currentAmp));
    setSlavesFollow();
  }

  @Override
  public void setVoltage(double voltageVolt) {
    master.setControl(voltageSetter.withOutput(voltageVolt));
    setSlavesFollow();
  }

  @Override
  public void stop() {
    master.setControl(neutralSetter);
    setSlavesFollow();
  }

  private void setSlavesFollow() {
    for (int i = 0; i < slaves.size(); i++) {
      slaves.get(i).setControl(slaveSetters.get(i));
    }
  }
}
