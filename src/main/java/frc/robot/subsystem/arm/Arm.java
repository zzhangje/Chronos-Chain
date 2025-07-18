package frc.robot.subsystem.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.dashboard.Alert;
import frc.lib.interfaces.motor.GenericArmIO;
import frc.lib.interfaces.motor.GenericArmIOInputsAutoLogged;
import frc.lib.interfaces.motor.GenericArmIOKrakenCancoder;
import frc.lib.interfaces.motor.GenericArmIOSim;
import frc.lib.interfaces.motor.GenericElevatorIO;
import frc.lib.interfaces.motor.GenericElevatorIOInputsAutoLogged;
import frc.lib.interfaces.motor.GenericElevatorIOKraken;
import frc.lib.interfaces.motor.GenericElevatorIOSim;
import frc.lib.interfaces.motor.GenericRollerIO;
import frc.lib.interfaces.motor.GenericRollerIOInputsAutoLogged;
import frc.lib.interfaces.motor.GenericRollerIOKraken;
import frc.lib.interfaces.motor.GenericRollerIOSim;
import frc.lib.interfaces.sensor.digital.BiDigitalInputCandi;
import frc.robot.Constants.Ports;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;
import frc.robot.subsystem.arm.ArmGoal.EndEffectorGoal;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Arm {
  static {
    final var shouldeGains = ArmConfig.getShoulderGains();
    ArmConfig.shoulderKp.initDefault(shouldeGains.kp());
    ArmConfig.shoulderKd.initDefault(shouldeGains.kd());
    ArmConfig.shoulderKs.initDefault(shouldeGains.ks());
    ArmConfig.shoulderKg.initDefault(shouldeGains.kg());

    final var elbowGains = ArmConfig.getElbowGains();
    ArmConfig.elbowKp.initDefault(elbowGains.kp());
    ArmConfig.elbowKd.initDefault(elbowGains.kd());
    ArmConfig.elbowKs.initDefault(elbowGains.ks());
    ArmConfig.elbowKg.initDefault(elbowGains.kg());
  }

  private final GenericElevatorIO shoulderIO;
  private final GenericArmIO elbowIO;
  private final GenericRollerIO eeIO;

  private final GenericElevatorIOInputsAutoLogged shoulderInputs =
      new GenericElevatorIOInputsAutoLogged();
  private final GenericArmIOInputsAutoLogged elbowInputs = new GenericArmIOInputsAutoLogged();
  private final GenericRollerIOInputsAutoLogged eeInputs = new GenericRollerIOInputsAutoLogged();

  private final Alert shoulderOfflineAlert =
      new Alert("Arm shoulder motor offline!", Alert.AlertType.WARNING);
  private final Alert elbowOfflineAlert =
      new Alert("Arm elbow motor offline!", Alert.AlertType.WARNING);
  private final Alert eeOfflineAlert =
      new Alert("Arm end effector motor offline!", Alert.AlertType.WARNING);

  @Getter @AutoLogOutput private ArmSubsystemGoal armGoal = ArmSubsystemGoal.IDLE;
  @Getter @AutoLogOutput private EndEffectorGoal eeGoal = EndEffectorGoal.IDLE;

  private final BooleanSupplier hasCoralSupplier, hasAlgaeSupplier;

  private Arm(
      GenericElevatorIO shoulderIO,
      GenericArmIO elbowIO,
      GenericRollerIO eeIO,
      BooleanSupplier hasCoralSupplier,
      BooleanSupplier hasAlgaeSupplier) {
    this.shoulderIO = shoulderIO;
    this.elbowIO = elbowIO;
    this.eeIO = eeIO;
    this.hasCoralSupplier = hasCoralSupplier;
    this.hasAlgaeSupplier = hasAlgaeSupplier;
  }

  public static Arm createReal() {
    BiDigitalInputCandi rollerSensor =
        new BiDigitalInputCandi(
            "RollerSensor", Ports.Can.END_EFFECTOR_CANDI, ArmConfig.getRollerSensorConfig());
    return new Arm(
        new GenericElevatorIOKraken(
                "ArmShoulder",
                Ports.Can.ARM_SHOULDER_MASTER,
                ArmConfig.getShoulderTalonConfig(),
                ArmConfig.SHOULDER_INIT_HEIGHT_METER,
                ArmConfig.SHOULDER_METER_PER_ROTATION)
            .withFollower(Ports.Can.ARM_SHOULDER_SLAVE, true),
        new GenericArmIOKrakenCancoder(
            "ArmElbow",
            Ports.Can.ARM_ELBOW,
            ArmConfig.getElbowTalonConfig(),
            Ports.Can.ARM_ELBOW_CANCODER,
            true,
            ArmConfig.ELBOW_CANCODER_OFFSET,
            0.75),
        new GenericRollerIOKraken(
            "EndEffector", Ports.Can.END_EFFECTOR, ArmConfig.getEndEffectorTalonConfig()),
        () -> !rollerSensor.getValue1(),
        () -> !rollerSensor.getValue2());
  }

  public static Arm createSim(BooleanSupplier hasCoralSupplier, BooleanSupplier hasAlgaeSupplier) {
    return new Arm(
        new GenericElevatorIOSim(
            DCMotor.getKrakenX60Foc(2),
            ArmConfig.SHOULDER_REDUCTION,
            7.5,
            ArmConfig.SHOULDER_METER_PER_ROTATION,
            ArmConfig.SHOULDER_MIN_HEIGHT_METER,
            ArmConfig.SHOULDER_MAX_HEIGHT_METER,
            ArmSubsystemGoal.START.getShoulderHeightMeter()),
        new GenericArmIOSim(
            DCMotor.getKrakenX60Foc(1),
            ArmConfig.ELBOW_REDUCTION,
            0.46,
            0.1,
            Units.degreesToRadians(-360),
            Units.degreesToRadians(360.0),
            ArmSubsystemGoal.START.getElbowPositionRad()),
        new GenericRollerIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.001),
        hasCoralSupplier,
        hasAlgaeSupplier);
  }

  public static Arm createIO() {
    return new Arm(
        new GenericElevatorIO() {},
        new GenericArmIO() {},
        new GenericRollerIO() {},
        () -> false,
        () -> false);
  }

  public double getCarriageHeightMeter() {
    double singleLevelHeightMeter = ArmConfig.SHOULDER_MAX_HEIGHT_METER / 2.0;
    return MathUtil.clamp(shoulderInputs.positionMeter, 0.0, singleLevelHeightMeter);
  }

  public double getElevatorHeightMeter() {
    double singleLevelHeightMeter = ArmConfig.SHOULDER_MAX_HEIGHT_METER / 2.0;
    return MathUtil.clamp(
        shoulderInputs.positionMeter - singleLevelHeightMeter, 0.0, singleLevelHeightMeter);
  }

  public double getElbowPositionRad() {
    return elbowInputs.positionRad;
  }
}
