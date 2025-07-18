package frc.robot.subsystem.arm;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.DebugGroup;

class ArmConfig {
  static final double ELBOW_CANCODER_OFFSET = 0.22314453125;
  static final double ELBOW_REDUCTION = (74.0 / 12.0) * (60.0 / 20.0) * (64.0 / 16.0);
  protected static final LoggedTunableNumber transitionElevatorHeightMeter =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/TransitionElevatorHeightMeter", 0.85);
  protected static final LoggedTunableNumber minSafeGroundIntakeDodgeElevatorHeightMeter =
      new LoggedTunableNumber(
          DebugGroup.ARM, "Arm/MinSafeGroundIntakeDodgeElevatorHeightMeter", 0.8);
  protected static final LoggedTunableNumber maxUnsafeGroundIntakeDodgeElevatorHeightMeter =
      new LoggedTunableNumber(
          DebugGroup.ARM, "Arm/MaxUnsafeGroundIntakeDodgeElevatorHeightMeter", 0.3);

  protected static final LoggedTunableNumber shoulderMotionMagicVelMeterPerSec =
      new LoggedTunableNumber("Arm/Shoulder/MotionMagicVelMeterPerSec", 6.0);
  protected static final LoggedTunableNumber shoulderMotionMagicAccelMeterPerSec2 =
      new LoggedTunableNumber("Arm/Shoulder/MotionMagicAccelMeterPerSec2", 20.0);
  protected static final LoggedTunableNumber shoulderKp =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Kp");
  protected static final LoggedTunableNumber shoulderKd =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Kd");
  protected static final LoggedTunableNumber shoulderKs =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Ks");
  protected static final LoggedTunableNumber shoulderKg =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Kg");
  protected static final LoggedTunableNumber shoulderToleranceMeter =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/ToleranceMeter", 0.07);
  protected static final LoggedTunableNumber shoulderStopToleranceMeterPerSec =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/StopToleranceMeterPerSec", 0.1);
  protected static final LoggedTunableNumber
      shoulderStaticCharacterizationVelocityThreshMeterPerSec =
          new LoggedTunableNumber(
              DebugGroup.ARM, "Arm/Shoulder/StaticCharacterizationVelocityThreshMeterPerSec", 0.01);
  protected static final LoggedTunableNumber shoulderHomingCurrentAmp =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/HomingCurrentAmp", -3.0);
  protected static final LoggedTunableNumber shoulderHomingTimeSecs =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/HomingTimeSecs", 0.2);
  protected static final LoggedTunableNumber shoulderHomingVelocityThreshMeterPerSec =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/HomingVelocityThreshMeterPerSec", 0.05);

  protected static final LoggedTunableNumber elbowMotionMagicVelDegreePerSec =
      new LoggedTunableNumber("Arm/Elbow/MotionMagicVelDegreePerSec", 500.0);
  protected static final LoggedTunableNumber elbowMotionMagicAccelDegreePerSec2 =
      new LoggedTunableNumber("Arm/Elbow/MotionMagicAccelDegreePerSec2", 3000.0);
  protected static final LoggedTunableNumber elbowKp =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Kp");
  protected static final LoggedTunableNumber elbowKd =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Kd");
  protected static final LoggedTunableNumber elbowKs =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Ks");
  protected static final LoggedTunableNumber elbowKg =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Kg");
  protected static final LoggedTunableNumber elbowToleranceDegree =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/ToleranceDegree", 7.0);
  protected static final LoggedTunableNumber elbowStopToleranceDegreePerSec =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/StopToleranceDegreePerSec", 10.0);
  protected static final LoggedTunableNumber elbowStaticCharacterizationVelocityThreshDegreePerSec =
      new LoggedTunableNumber(
          DebugGroup.ARM, "Arm/Elbow/StaticCharacterizationVelocityThreshDegreePerSec", 10.0);
  protected static final LoggedTunableNumber elbowAvoidReefAlgaePositionDegree =
      new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/AvoidReefAlgaePositionDegree", 90.0);

  static Gains getElbowGains() {
    return switch (Constants.MODE) {
      case REAL -> new Gains(1100.0, 180.0, 2.5, 0.0);
      case SIM, REPLAY -> new Gains(10.0, 6, 0.0, 0.0);
    };
  }

  static TalonFXConfiguration getElbowTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var gains = getElbowGains();
    config.Slot0 =
        new Slot0Configs()
            .withKP(gains.kp())
            .withKD(gains.kd())
            .withKS(gains.ks())
            .withKG(gains.kg())
            .withGravityType(GravityTypeValue.Arm_Cosine);

    config.TorqueCurrent.PeakForwardTorqueCurrent = 170.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -170.0;

    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 100.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.RotorToSensorRatio = ELBOW_REDUCTION;

    config.ClosedLoopGeneral.ContinuousWrap = false;

    return config;
  }

  static final double SHOULDER_INIT_HEIGHT_METER = 0.0;
  static final double SHOULDER_MAX_HEIGHT_METER = 1.505;
  static final double SHOULDER_MIN_HEIGHT_METER = 0.0;
  static final double SHOULDER_METER_PER_ROTATION = 1.505 / 12.25;
  static final double SHOULDER_REDUCTION = 40.0 / 12.0;

  static Gains getShoulderGains() {
    return switch (Constants.MODE) {
        // case REAL -> new Gains(300.0, 30.0, 0.0, 13.0); // FIXME: Verify this value
      case REAL -> new Gains(100.0, 10.0, 2.0, 14.0); // Kg Ks Done
      case SIM, REPLAY -> new Gains(160.0, 2.7, 0.0, 0.0);
    };
  }

  static TalonFXConfiguration getShoulderTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var gains = getShoulderGains();
    config.Slot0 =
        new Slot0Configs()
            .withKP(gains.kp())
            .withKD(gains.kd())
            .withKS(gains.ks())
            .withKG(gains.kg())
            .withGravityType(GravityTypeValue.Elevator_Static);

    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 100.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = SHOULDER_REDUCTION;

    return config;
  }

  static TalonFXConfiguration getEndEffectorTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    return config;
  }

  static CANdiConfiguration getRollerSensorConfig() {
    var config = new CANdiConfiguration();
    config.DigitalInputs.S2FloatState = S2FloatStateValue.PullHigh;
    return config;
  }

  record Gains(double kp, double kd, double ks, double kg) {}
}
