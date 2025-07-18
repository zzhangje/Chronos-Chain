package frc.robot.subsystem.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.dashboard.LoggedTunableNumber;

public class ClimberConfig {
  protected static final LoggedTunableNumber lickVoltage =
      new LoggedTunableNumber("Climber/LickVoltage", 3.0);
  protected static final LoggedTunableNumber pullVoltage =
      new LoggedTunableNumber("Climber/PullVoltage", 3.0);

  protected static final LoggedTunableNumber readyPositionDeg =
      new LoggedTunableNumber("Climber/ReadyPositionDeg", 90.0);
  protected static final LoggedTunableNumber pullPositionDeg =
      new LoggedTunableNumber("Climber/PullPositionDeg", 180.0);

  protected static double HOME_POSITION_DEGREE = -15;
  protected static double REDUCTION = 76.83 * 4;

  static TalonFXConfiguration getPullTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.StatorCurrentLimit = 250.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = REDUCTION;

    return config;
  }

  static TalonFXConfiguration getLickTalonConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // FIXME: Verify this value

    config.CurrentLimits.StatorCurrentLimit = 250.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    return config;
  }
}
