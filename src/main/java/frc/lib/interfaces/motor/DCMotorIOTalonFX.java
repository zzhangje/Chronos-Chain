package frc.lib.interfaces.motor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.dashboard.Alert;
import frc.lib.interfaces.CanId;
import frc.lib.math.UnitConverter;
import frc.lib.utils.Phoenix6Helper;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;

public class DCMotorIOTalonFX implements DCMotorIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config;
  private final Alert offlineAlert;
  private CANcoder cancoder;
  private final List<TalonFX> slaves = new ArrayList<>();

  private UnitConverter ratioConverter = UnitConverter.identity();
  private UnitConverter positionConvertor = UnitConverter.identity();

  @Getter private final StatusSignal<Angle> rawPositionSignal;
  @Getter private final StatusSignal<AngularVelocity> rawVelocitySignal;
  @Getter private final StatusSignal<AngularAcceleration> rawAccelerationSignal;
  @Getter private final StatusSignal<Voltage> outputVoltageSignal;
  @Getter private final StatusSignal<Current> statorCurrentSignal;
  @Getter private final StatusSignal<Current> supplyCurrentSignal;
  @Getter private final StatusSignal<Temperature> temperatureSignal;

  public DCMotorIOTalonFX(
      String name,
      CanId talonfx,
      TalonFXConfiguration config,
      UnitConverter ratioConverter,
      UnitConverter... offsetConverter) {
    this.motor = new TalonFX(talonfx.id(), talonfx.bus());
    this.config = config;
    setUnitConvertor(ratioConverter, offsetConverter);

    var wrappedName = "[" + name + "]";
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " clear sticky fault", motor::clearStickyFaults);
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " config", () -> motor.getConfigurator().apply(config));

    offlineAlert = new Alert(wrappedName + " offline!", Alert.AlertType.WARNING);

    rawPositionSignal = motor.getPosition();
    rawVelocitySignal = motor.getVelocity();
    rawAccelerationSignal = motor.getAcceleration();
    outputVoltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    supplyCurrentSignal = motor.getSupplyCurrent();
    temperatureSignal = motor.getDeviceTemp();

    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " set signals update frequency",
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                rawPositionSignal,
                rawVelocitySignal,
                rawAccelerationSignal,
                outputVoltageSignal,
                supplyCurrentSignal,
                statorCurrentSignal,
                temperatureSignal));
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " optimize CAN utilization", motor::optimizeBusUtilization);
  }

  public void updateInputs(DCMotorIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                rawPositionSignal,
                rawVelocitySignal,
                rawAccelerationSignal,
                outputVoltageSignal,
                supplyCurrentSignal,
                statorCurrentSignal,
                temperatureSignal)
            .isOK();

    offlineAlert.set(!inputs.connected);

    // mechanism, rotated units
    inputs.rawPosition = rawPositionSignal.getValueAsDouble();
    inputs.rawVelocity = rawVelocitySignal.getValueAsDouble();
    inputs.rawAcceleration = rawAccelerationSignal.getValueAsDouble();
    inputs.rawUnit = ratioConverter.getFromUnits();

    // mechanism, applied units
    inputs.appliedPosition = positionConvertor.applyAsDouble(inputs.rawPosition);
    inputs.appliedVelocity = ratioConverter.applyAsDouble(inputs.rawVelocity);
    inputs.appliedAcceleration = ratioConverter.applyAsDouble(inputs.rawAcceleration);
    inputs.appliedUnit = ratioConverter.getToUnits();

    inputs.outputVoltageVolts = outputVoltageSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
    inputs.temperatureCelsius = temperatureSignal.getValueAsDouble();
  }

  @Override
  public void setPidsg(double kp, double ki, double kd, double ks, double kg) {
    config.Slot0.kP = kp;
    config.Slot0.kI = ki;
    config.Slot0.kD = kd;
    config.Slot0.kS = ks;
    config.Slot0.kG = kg;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setPid(double kp, double ki, double kd) {
    config.Slot0.kP = kp;
    config.Slot0.kI = ki;
    config.Slot0.kD = kd;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setMotionConstraints(double maxVelocityRadPerSec, double maxAccelerationRadPerSecSq) {
    // TODO
  }

  @Override
  public void setRotationContinuous(boolean isContinuous) {
    config.ClosedLoopGeneral.ContinuousWrap = isContinuous;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setUnitConvertor(UnitConverter ratioConverter, UnitConverter... offsetConverter) {
    this.ratioConverter = ratioConverter;
    if (offsetConverter.length > 0) {
      this.positionConvertor = ratioConverter.andThen(offsetConverter[0]);
    } else {
      this.positionConvertor = ratioConverter;
    }
  }

  @Override
  public void setAppliedPositionF(
      double position, double velocity, double acceleration, double feedforward) {
    motor.setControl(
        new DynamicMotionMagicTorqueCurrentFOC(
                ratioConverter.convertInverse(position),
                ratioConverter.convertInverse(velocity),
                ratioConverter.convertInverse(acceleration),
                0.0)
            .withFeedForward(feedforward));
  }

  @Override
  public void setAppliedPosition(double position, double velocity, double acceleration) {
    motor.setControl(
        new PositionTorqueCurrentFOC(ratioConverter.convertInverse(position))
            .withVelocity(ratioConverter.convertInverse(velocity)));
  }

  @Override
  public void setAppliedPositionF(double position, double feedforward) {
    motor.setControl(
        new PositionTorqueCurrentFOC(ratioConverter.convertInverse(position))
            .withFeedForward(feedforward));
  }

  @Override
  public void setAppliedVelocityF(double velocity, double acceleration, double feedforward) {
    motor.setControl(
        new MotionMagicVelocityTorqueCurrentFOC(ratioConverter.convertInverse(velocity))
            .withAcceleration(ratioConverter.convertInverse(acceleration))
            .withFeedForward(feedforward));
  }

  @Override
  public void setAppliedVelocityF(double velocity, double feedforward) {
    motor.setControl(
        new VelocityTorqueCurrentFOC(ratioConverter.convertInverse(velocity))
            .withFeedForward(feedforward));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setCurrent(double amps) {
    motor.setControl(new TorqueCurrentFOC(amps));
  }

  @Override
  public void resetRawPosition(double position) {
    motor.setPosition(position);
  }

  @Override
  public void resetAppliedPosition(double position) {
    motor.setPosition(positionConvertor.convertInverse(position));
  }

  @Override
  public double getVoltage() {
    return outputVoltageSignal.getValueAsDouble();
  }

  @Override
  public double getCurrent() {
    return supplyCurrentSignal.getValueAsDouble();
  }

  @Override
  public double getAppliedPosition() {
    return positionConvertor.applyAsDouble(rawPositionSignal.getValueAsDouble());
  }

  @Override
  public double getAppliedVelocity() {
    return ratioConverter.applyAsDouble(rawVelocitySignal.getValueAsDouble());
  }

  @Override
  public double getAppliedAcceleration() {
    return ratioConverter.applyAsDouble(rawAccelerationSignal.getValueAsDouble());
  }

  @Override
  public double getRawPosition() {
    return rawPositionSignal.getValueAsDouble();
  }

  @Override
  public double getRawVelocity() {
    return rawVelocitySignal.getValueAsDouble();
  }

  @Override
  public double getRawAcceleration() {
    return rawAccelerationSignal.getValueAsDouble();
  }

  @Override
  public int getDeviceID() {
    return motor.getDeviceID();
  }

  public DCMotorIOTalonFX withCancoder(
      String name, CanId cancoder, CANcoderConfiguration coderConfig) {
    var wrappedName = "[" + name + "]";
    this.cancoder = new CANcoder(cancoder.id(), cancoder.bus());
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " CANCoder config", () -> this.cancoder.getConfigurator().apply(coderConfig));

    config.Feedback.FeedbackRemoteSensorID = cancoder.id();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    Phoenix6Helper.checkErrorAndRetry(
        wrappedName + " config remote cancoder mode", () -> motor.getConfigurator().apply(config));

    return this;
  }

  public DCMotorIOTalonFX withFollower(
      CanId talonfx, TalonFXConfiguration config, boolean isInverted) {
    // the main purpose of the config
    // is to set the motor mode when disabled
    TalonFX slave = new TalonFX(talonfx.id(), talonfx.bus());
    slave.setControl(new Follower(getDeviceID(), isInverted));
    slave.getConfigurator().apply(config);
    slaves.add(slave);
    return this;
  }
}
