package frc.robot.subsystem.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;

public class SwerveConfig {
  static final String FL_MODULE_NAME = "FL";
  static final String BL_MODULE_NAME = "BL";
  static final String BR_MODULE_NAME = "BR";
  static final String FR_MODULE_NAME = "FR";

  static final double ODOMETRY_FREQUENCY_HZ = 250.0;

  static final boolean ENABLE_TRAJECTORY_FF = true;

  public static final double WHEELBASE_LENGTH_METER = 0.56705;
  public static final double WHEELBASE_WIDTH_METER = 0.56705;
  public static final double WHEELBASE_DIAGONAL_METER =
      Math.hypot(WHEELBASE_LENGTH_METER, WHEELBASE_WIDTH_METER);

  public static final double MAX_TRANSLATION_VEL_METER_PER_SEC = 5.0292; // MK4i L3 Gear Ratio
  public static final double MAX_ANGULAR_VEL_RAD_PER_SEC =
      MAX_TRANSLATION_VEL_METER_PER_SEC / (WHEELBASE_DIAGONAL_METER / 2.0);

  public static final SwerveDriveKinematics SWERVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEELBASE_LENGTH_METER / 2.0, WHEELBASE_WIDTH_METER / 2.0),
          new Translation2d(-WHEELBASE_LENGTH_METER / 2.0, WHEELBASE_WIDTH_METER / 2.0),
          new Translation2d(-WHEELBASE_LENGTH_METER / 2.0, -WHEELBASE_WIDTH_METER / 2.0),
          new Translation2d(WHEELBASE_LENGTH_METER / 2.0, -WHEELBASE_WIDTH_METER / 2.0));

  static final double WHEEL_RADIUS_METER = 0.0479;
  static final ModuleConfig FL_MODULE_CONFIG =
      new ModuleConfig(
          getX2iDriveTalonConfig(),
          getX2iSteerTalonNoEncoderConfig(),
          getCancoderConfig(-0.3876953125));
  static final ModuleConfig BL_MODULE_CONFIG =
      new ModuleConfig(
          getX2iDriveTalonConfig(),
          getX2iSteerTalonNoEncoderConfig(),
          getCancoderConfig(0.3974609375));
  static final ModuleConfig BR_MODULE_CONFIG =
      new ModuleConfig(
          getX2iDriveTalonConfig(),
          getX2iSteerTalonNoEncoderConfig(),
          getCancoderConfig(-0.37451171875));
  static final ModuleConfig FR_MODULE_CONFIG =
      new ModuleConfig(
          getX2iDriveTalonConfig(),
          getX2iSteerTalonNoEncoderConfig(),
          getCancoderConfig(0.40869140625));

  /// WCP X2i Reductions
  /// https://docs.wcproducts.com/wcp-swerve-x2/general-info/ratio-options
  /// - X1T10: (54.0 / 10.0) * (18.0 / 38.0) * (45.0 / 15.0) = 7.67
  /// - X1T11: (54.0 / 11.0) * (18.0 / 38.0) * (45.0 / 15.0) = 6.98
  /// - X1T12: (54.0 / 12.0) * (18.0 / 38.0) * (45.0 / 15.0) = 6.39
  /// - X2T10: (54.0 / 10.0) * (16.0 / 38.0) * (45.0 / 15.0) = 6.82
  /// - X2T11: (54.0 / 11.0) * (16.0 / 38.0) * (45.0 / 15.0) = 6.20
  /// - X2T12: (54.0 / 12.0) * (16.0 / 38.0) * (45.0 / 15.0) = 5.68
  /// - X3T10: (54.0 / 10.0) * (16.0 / 40.0) * (45.0 / 15.0) = 6.48
  /// - X3T11: (54.0 / 11.0) * (16.0 / 40.0) * (45.0 / 15.0) = 5.89
  /// - X3T12: (54.0 / 12.0) * (16.0 / 40.0) * (45.0 / 15.0) = 5.40
  /// - X4T10: (54.0 / 10.0) * (14.0 / 40.0) * (45.0 / 15.0) = 5.67
  /// - X4T11: (54.0 / 11.0) * (14.0 / 40.0) * (45.0 / 15.0) = 5.15
  /// - X4T12: (54.0 / 12.0) * (14.0 / 40.0) * (45.0 / 15.0) = 4.73
  /// - TURN: (88.0 / 16.0) * (22.0 / 27.0) * (27.0 / 10.0) = 12.1
  ///
  /// MK4i Reductions
  /// https://cdn.shopify.com/s/files/1/0065/4308/1590/files/14T_Gear_Ratios.png
  /// (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0); // MK4i L3
  /// (50.0 / 14.0) * (60.0 / 10.0); // MK4i L3
  static final double DRIVE_REDUCTION = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  static final double STEER_REDUCTION = (50.0 / 14.0) * (60.0 / 10.0);

  static final double DRIVE_FF_KT =
      DCMotor.getKrakenX60Foc(1).withReduction(DRIVE_REDUCTION).KtNMPerAmp;

  record ModuleConfig(
      TalonFXConfiguration driveTalonConfig,
      TalonFXConfiguration steerTalonConfig,
      CANcoderConfiguration cancoderConfig) {}

  static Gains getDriveGains() {
    return switch (Constants.MODE) {
      case REAL -> new Gains(7.0, 0.0, 0.0);
      case SIM, REPLAY -> new Gains(0.3, 0.0, 0.0);
    };
  }

  static Gains getSteerGains() {
    return switch (Constants.MODE) {
      case REAL -> new Gains(2000.0, 50.0, 0.0);
      case SIM, REPLAY -> new Gains(10.0, 0.0, 0.0);
    };
  }

  record Gains(double kp, double kd, double ks) {}

  static TalonFXConfiguration getX2iDriveTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var gains = getDriveGains();
    config.Slot0 = new Slot0Configs().withKP(gains.kp()).withKD(gains.kd()).withKS(gains.ks());

    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 200.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = DRIVE_REDUCTION;

    return config;
  }

  static TalonFXConfiguration getX2iSteerTalonNoEncoderConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var gains = getSteerGains();
    config.Slot0 = new Slot0Configs().withKP(gains.kp()).withKD(gains.kd()).withKS(gains.ks());
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = 100.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.RotorToSensorRatio = STEER_REDUCTION;

    config.ClosedLoopGeneral.ContinuousWrap = true;

    return config;
  }

  private static CANcoderConfiguration getCancoderConfig(double magnetOffset) {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = magnetOffset;

    return config;
  }
}
