package frc.lib.interfaces.motor;

import frc.lib.math.GainsUtil.Gains;
import frc.lib.math.UnitConverter;
import org.littletonrobotics.junction.AutoLog;

public interface DCMotorIO {
  @AutoLog
  class DCMotorIOInputs {
    // Connection status
    public boolean connected = false;

    // Raw sensor data (angular units)
    public double rawPosition = 0.0;
    public double rawVelocity = 0.0;
    public double rawAcceleration = 0.0;
    public String rawUnit = "";

    // Mechanism data (applied units)
    public double appliedPosition = 0.0;
    public double appliedVelocity = 0.0;
    public double appliedAcceleration = 0.0;
    public String appliedUnit = "";

    // Electrical measurements
    public double outputVoltageVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  // ========== Configuration Methods ==========

  /**
   * Configures PIDF gains for the motor controller.
   *
   * @param kp Proportional gain
   * @param ki Integral gain (optional, default 0)
   * @param kd Derivative gain (optional, default 0)
   * @param ks Static feedforward gain
   * @param kg Gravity feedforward gain
   */
  default void setPidsg(double kp, double ki, double kd, double ks, double kg) {}

  default void setGains(Gains gains) {
    setPidsg(gains.getKP(), gains.getKI(), gains.getKD(), gains.getKS(), gains.getKG());
  }

  /**
   * Configures PID gains for the motor controller.
   *
   * @param kp Proportional gain
   * @param ki Integral gain (optional, default 0)
   * @param kd Derivative gain (optional, default 0)
   */
  default void setPid(double kp, double ki, double kd) {
    setPidsg(kp, ki, kd, 0.0, 0.0);
  }

  /**
   * Configures PD gains for the motor controller.
   *
   * @param kp Proportional gain
   * @param kd Derivative gain
   * @param ks Static feedforward gain
   */
  default void setPds(double kp, double kd, double ks) {
    setPidsg(kp, 0.0, kd, ks, 0.0);
  }

  /**
   * Sets motion profile constraints for smart motion control.
   *
   * @param maxVelocity Maximum velocity in radians per second
   * @param maxAcceleration Maximum acceleration in radians per second squared
   */
  default void setMotionConstraints(double maxVelocity, double maxAcceleration) {}

  /**
   * Sets the motor to operate in continuous rotation mode.
   *
   * @param isContinuous Whether to enable continuous rotation mode
   */
  default void setRotationContinuous(boolean isContinuous) {}

  /**
   * Sets the unit conversion for position and velocity.
   *
   * @param ratioConverter Converter for position, velocity, and acceleration
   * @param offsetConverter Converter for position (optional)
   */
  default void setUnitConvertor(UnitConverter ratioConverter, UnitConverter... offsetConverter) {}

  // ========== Control Methods ==========

  /**
   * Sets target position with full motion control parameters.
   *
   * @param position Target position in radians
   * @param velocity Target velocity in radians per second
   * @param acceleration Target acceleration in radians per second squared
   * @param feedforward Additional feedforward voltage
   */
  default void setAppliedPositionF(
      double position, double velocity, double acceleration, double feedforward) {}

  /**
   * Sets target position with velocity and acceleration parameters.
   *
   * @param position Target position in radians
   * @param velocity Target velocity in radians per second
   * @param acceleration Target acceleration in radians per second squared
   */
  default void setAppliedPosition(double position, double velocity, double acceleration) {
    setAppliedPositionF(position, velocity, acceleration, 0.0);
  }

  /**
   * Sets target position with feedforward voltage.
   *
   * @param position Target position in radians
   * @param feedforward Additional feedforward voltage
   */
  default void setAppliedPositionF(double position, double feedforward) {
    setAppliedPositionF(position, 0, 0, feedforward);
  }

  /**
   * Sets target position with basic control.
   *
   * @param position Target position in radians
   */
  default void setAppliedPosition(double position) {
    setAppliedPositionF(position, 0);
  }

  /**
   * Sets target velocity with full motion control parameters.
   *
   * @param velocity Target velocity in radians per second
   * @param acceleration Target acceleration in radians per second squared
   * @param feedforward Additional feedforward voltage
   */
  default void setAppliedVelocityF(double velocity, double acceleration, double feedforward) {}

  /**
   * Sets target velocity with acceleration control.
   *
   * @param velocity Target velocity in radians per second
   * @param acceleration Target acceleration in radians per second squared
   */
  default void setAppliedVelocity(double velocity, double acceleration) {
    setAppliedVelocityF(velocity, acceleration, 0.0);
  }

  /**
   * Sets target velocity with feedforward voltage.
   *
   * @param velocity Target velocity in radians per second
   * @param feedforward Additional feedforward voltage
   */
  default void setAppliedVelocityF(double velocity, double feedforward) {
    setAppliedVelocityF(velocity, 0, feedforward);
  }

  /**
   * Sets target velocity with basic control.
   *
   * @param velocity Target velocity in radians per second
   */
  default void setAppliedVelocity(double velocity) {
    setAppliedVelocityF(velocity, 0);
  }

  /**
   * Sets direct voltage output to motor.
   *
   * @param volts Voltage to apply (-12 to 12V)
   */
  default void setVoltage(double volts) {}

  /**
   * Sets current limit for the motor.
   *
   * @param amps Current limit in amps
   */
  default void setCurrent(double amps) {}

  /**
   * Resets the motor position sensor.
   *
   * @param position New position in radians
   */
  default void resetAppliedPosition(double position) {}

  /** */
  default void resetRawPosition(double position) {}

  /** Stop the motor */
  default void stop() {
    setVoltage(0);
  }

  // ========== Status Methods ==========

  /**
   * Updates the provided inputs object with current sensor data.
   *
   * @param inputs Inputs object to populate with current data
   */
  default void updateInputs(DCMotorIOInputs inputs) {}

  /**
   * Gets the current output voltage of the motor.
   *
   * @return Current output voltage in volts
   */
  default double getVoltage() {
    return 0.0;
  }

  /**
   * Gets the current supply current of the motor.
   *
   * @return Current supply current in amps
   */
  default double getCurrent() {
    return 0.0;
  }

  /** */
  default double getAppliedPosition() {
    return 0.0;
  }

  default double getAppliedVelocity() {
    return 0.0;
  }

  default double getAppliedAcceleration() {
    return 0.0;
  }

  default double getRawPosition() {
    return 0.0;
  }

  default double getRawVelocity() {
    return 0.0;
  }

  default double getRawAcceleration() {
    return 0.0;
  }

  /**
   * Gets the device ID of the motor controller.
   *
   * @return Device ID (default implementation returns 0)
   */
  default int getDeviceID() {
    return 0;
  }
}
