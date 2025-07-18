package frc.lib.interfaces.motor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.math.GainsUtil.Gains;
import frc.lib.math.UnitConverter;

public class DCMotorIOSim implements DCMotorIO {
  private final DCMotorSim sim;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          0,
          0,
          0,
          new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
  private final SlewRateLimiter voltageLimiter = new SlewRateLimiter(1000);

  private UnitConverter ratioConverter = UnitConverter.identity();
  private UnitConverter positionConvertor = UnitConverter.identity();

  private double minRawPosition = -Double.MAX_VALUE;
  private double maxRawPosition = Double.MAX_VALUE;

  @Override
  public void updateInputs(DCMotorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    sim.update(0.02);

    inputs.connected = true;

    if (sim.getAngularPositionRad() < minRawPosition) {
      sim.setState(minRawPosition, 0);
    } else if (sim.getAngularPositionRad() > maxRawPosition) {
      sim.setState(maxRawPosition, 0);
    }

    // rotor, radians
    inputs.rawPosition = sim.getAngularPositionRad();
    inputs.rawVelocity = sim.getAngularVelocityRadPerSec();
    inputs.rawAcceleration = sim.getAngularAccelerationRadPerSecSq();
    inputs.rawUnit = ratioConverter.getFromUnits();

    // mechanism, applied units
    inputs.appliedPosition = positionConvertor.applyAsDouble(inputs.rawPosition);
    inputs.appliedVelocity = ratioConverter.applyAsDouble(inputs.rawVelocity);
    inputs.appliedAcceleration = ratioConverter.applyAsDouble(inputs.rawAcceleration);
    inputs.appliedUnit = ratioConverter.getToUnits();

    inputs.outputVoltageVolts = sim.getInputVoltage();
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
    inputs.temperatureCelsius = 25.0; // Simulated temperature
  }

  @Override
  public void setPidsg(double kp, double ki, double kd, double ks, double kg) {
    pid.setPID(kp, ki, kd);
  }

  @Override
  public void setPid(double kp, double ki, double kd) {
    pid.setPID(kp, ki, kd);
  }

  @Override
  public void setMotionConstraints(double maxRawVelocity, double maxRawAcceleration) {
    pid.setConstraints(new TrapezoidProfile.Constraints(maxRawVelocity, maxRawAcceleration));
  }

  public void setAppliedPositionConstraints(double minAppliedPosition, double maxAppliedPosition) {
    minRawPosition = positionConvertor.convertInverse(minAppliedPosition);
    maxRawPosition = positionConvertor.convertInverse(maxAppliedPosition);
  }

  @Override
  public void setRotationContinuous(boolean isContinuous) {
    if (isContinuous) {
      pid.enableContinuousInput(-Math.PI, Math.PI);
    } else {
      pid.disableContinuousInput();
    }
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
    double pidOutput =
        pid.calculate(sim.getAngularPositionRad(), positionConvertor.convertInverse(position));
    setVoltage(pidOutput + feedforward);
  }

  @Override
  public void setAppliedVelocityF(double velocity, double acceleration, double feedforward) {
    double pidOutput =
        pid.calculate(sim.getAngularVelocityRadPerSec(), ratioConverter.convertInverse(velocity));
    setVoltage(pidOutput + feedforward);
  }

  @Override
  public void setVoltage(double volts) {
    if (DriverStation.isEnabled()) {
      sim.setInputVoltage(voltageLimiter.calculate(volts));
    } else {
      voltageLimiter.reset(0);
      sim.setInputVoltage(0);
    }
  }

  @Override
  public void setCurrent(double amps) {
    double resistance = 0.1; // Simulated motor resistance
    double kv = 0.01; // Simulated back-EMF constant
    double voltage = amps * resistance + sim.getAngularVelocityRadPerSec() * kv;
    setVoltage(voltage);
  }

  @Override
  public void resetRawPosition(double position) {
    sim.setAngle(position);
  }

  @Override
  public void resetAppliedPosition(double position) {
    sim.setAngle(positionConvertor.convertInverse(position));
  }

  @Override
  public double getVoltage() {
    return sim.getInputVoltage();
  }

  @Override
  public double getCurrent() {
    return sim.getCurrentDrawAmps();
  }

  @Override
  public double getAppliedPosition() {
    return positionConvertor.applyAsDouble(sim.getAngularPositionRad());
  }

  @Override
  public double getAppliedVelocity() {
    return ratioConverter.applyAsDouble(sim.getAngularVelocityRadPerSec());
  }

  @Override
  public double getAppliedAcceleration() {
    return ratioConverter.applyAsDouble(sim.getAngularAccelerationRadPerSecSq());
  }

  @Override
  public double getRawPosition() {
    return sim.getAngularPositionRad();
  }

  @Override
  public double getRawVelocity() {
    return sim.getAngularVelocityRadPerSec();
  }

  @Override
  public double getRawAcceleration() {
    return sim.getAngularAccelerationRadPerSecSq();
  }

  @Override
  public int getDeviceID() {
    return 0; // Simulation doesn't have device IDs
  }

  public DCMotorIOSim(
      DCMotor motor,
      double JKgMetersSquared,
      double gearing,
      UnitConverter ratioConverter,
      UnitConverter offsetConverter,
      Gains gains) {
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, JKgMetersSquared, gearing), motor);

    setUnitConvertor(ratioConverter, offsetConverter);
    setGains(gains);
  }

  public DCMotorIOSim(
      DCMotor motor,
      double JKgMetersSquared,
      double gearing,
      UnitConverter ratioConverter,
      Gains gains) {
    this(
        motor,
        JKgMetersSquared,
        gearing,
        ratioConverter,
        UnitConverter.identity()
            .withUnits(ratioConverter.getToUnits(), ratioConverter.getToUnits()),
        gains);
  }

  public DCMotorIOSim(
      LinearSystem<N2, N1, N2> system,
      DCMotor motor,
      UnitConverter ratioConverter,
      UnitConverter offsetConverter,
      Gains gains) {
    sim = new DCMotorSim(system, motor);
    setUnitConvertor(ratioConverter, offsetConverter);
    setGains(gains);
  }

  public DCMotorIOSim(
      LinearSystem<N2, N1, N2> system, DCMotor motor, UnitConverter ratioConverter, Gains gains) {
    this(
        system,
        motor,
        ratioConverter,
        UnitConverter.identity()
            .withUnits(ratioConverter.getToUnits(), ratioConverter.getToUnits()),
        gains);
  }

  public DCMotorIOSim(
      DCMotor motor,
      double JKgMetersSquared,
      double gearing,
      UnitConverter ratioConverter,
      UnitConverter offsetConverter,
      Gains gains,
      double minAppliedPosition,
      double maxAppliedPosition) {
    this(motor, JKgMetersSquared, gearing, ratioConverter, offsetConverter, gains);
    setAppliedPositionConstraints(minAppliedPosition, maxAppliedPosition);
  }

  public DCMotorIOSim(
      LinearSystem<N2, N1, N2> system,
      DCMotor motor,
      UnitConverter ratioConverter,
      UnitConverter offsetConverter,
      Gains gains,
      double minAppliedPosition,
      double maxAppliedPosition) {
    this(system, motor, ratioConverter, offsetConverter, gains);
    setAppliedPositionConstraints(minAppliedPosition, maxAppliedPosition);
  }

  public DCMotorIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.0001, 1), DCMotor.getNEO(1));
  }
}
