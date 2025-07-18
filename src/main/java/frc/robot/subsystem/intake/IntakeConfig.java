package frc.robot.subsystem.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.Constants;

public class IntakeConfig {
  protected static final double MIN_ANGLE_DEGREE = 0.0;
  protected static final double INIT_ANGLE_DEGREE = 0.0;
  protected static final double MAX_ANGLE_DEGREE = 130.0;

  protected static final LoggedTunableNumber TROUGH_ANGLE_DEGREE =
      new LoggedTunableNumber(Constants.DebugGroup.INTAKE, "Intake/TroughAngleDegree", 30.0);
  protected static final LoggedTunableNumber DODGE_ANGLE_DEGREE =
      new LoggedTunableNumber(Constants.DebugGroup.INTAKE, "Intake/DodgeAngleDegree", 20.0);

  protected static final LoggedTunableNumber TROUGH_ROLLER_VOLTAGE =
      new LoggedTunableNumber(Constants.DebugGroup.INTAKE, "Intake/TroughVoltage", 12);
  protected static final LoggedTunableNumber STANDBY_ROLLER_VOLTAGE =
      new LoggedTunableNumber(Constants.DebugGroup.INTAKE, "Intake/StandbyRollerVoltage", -1);
  protected static final LoggedTunableNumber INJECT_ROLLER_VOLTAGE =
      new LoggedTunableNumber(Constants.DebugGroup.INTAKE, "Intake/InjectRollerVoltage", -6);
  protected static final LoggedTunableNumber EJECT_ROLLER_VOLTAGE =
      new LoggedTunableNumber(Constants.DebugGroup.INTAKE, "Intake/EjectRollerVoltage", 12);

  protected static final LoggedTunableNumber STANDBY_CENTERING_VOLTAGE =
      new LoggedTunableNumber(Constants.DebugGroup.INTAKE, "Intake/StandbyCenteringVoltage", -2);
  protected static final LoggedTunableNumber INJECT_CENTERING_VOLTAGE =
      new LoggedTunableNumber(Constants.DebugGroup.INTAKE, "Intake/InjectCenteringVoltage", -12);

  protected static final double PIVOT_REDUCTION =
      (40.0 / 12.0) * (48.0 / 20.0) * (74.0 / 20.0) * (32.0 / 16.0);
  protected static final LoggedTunableNumber AT_SETPOINT_THRESHOLD =
      new LoggedTunableNumber(
          Constants.DebugGroup.INTAKE, "Intake/AtSetpointThresholdRadians", 0.1);
  protected static final LoggedTunableNumber IntakeMotionMagicVelMeterPerSec =
      new LoggedTunableNumber(
          Constants.DebugGroup.INTAKE, "Intake/MotionMagicVelMeterPerSec", 600.0);
  protected static final LoggedTunableNumber IntakeMotionMagicAccelMeterPerSec2 =
      new LoggedTunableNumber(
          Constants.DebugGroup.INTAKE, "Intake/MotionMagicAccelMeterPerSec2", 4000.0);

  static Gains getPivotGains() {
    return switch (Constants.MODE) {
      case REAL -> new Gains(130.0, 50.0, 0.0, 0.0); // Updated with calculated values
      case SIM, REPLAY -> new Gains(20.0, 0.4, 0.0, 0.0);
    };
  }

  static TalonFXConfiguration getRollerTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    return config;
  }

  static TalonFXConfiguration getPivotTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var gains = getPivotGains();
    config.Slot0 =
        new Slot0Configs()
            .withKP(gains.kp())
            .withKD(gains.kd())
            .withKS(gains.ks())
            .withKG(gains.kg())
            .withGravityType(GravityTypeValue.Arm_Cosine);

    config.TorqueCurrent.PeakForwardTorqueCurrent = 150.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -150.0;

    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = PIVOT_REDUCTION;

    config.ClosedLoopGeneral.ContinuousWrap = false;

    return config;
  }

  static TalonFXConfiguration getCenteringTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    return config;
  }

  record Gains(double kp, double kd, double ks, double kg) {}

  protected static final LoggedTunableNumber pivotKp =
      new LoggedTunableNumber("Intake", "Config/Kp");
  protected static final LoggedTunableNumber pivotKd =
      new LoggedTunableNumber("Intake", "Config/Kd");
  protected static final LoggedTunableNumber pivotKs =
      new LoggedTunableNumber("Intake", "Config/Ks");
  protected static final LoggedTunableNumber pivotKg =
      new LoggedTunableNumber("Intake", "Config/Kg");
}
