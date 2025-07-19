package frc.robot.subsystem.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.Alert;
import frc.lib.dashboard.LoggedTunableNumber;
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
import frc.lib.math.EqualsUtil;
import frc.lib.math.RotationUtil;
import frc.robot.Constants.Ports;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;
import frc.robot.subsystem.arm.ArmGoal.EndEffectorGoal;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
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
  @Getter @AutoLogOutput private boolean needIntakeDodge = false;

  @Override
  public void periodic() {
    shoulderIO.updateInputs(shoulderInputs);
    Logger.processInputs("Arm Shoulder", shoulderInputs);

    elbowIO.updateInputs(elbowInputs);
    Logger.processInputs("Arm Elbow", elbowInputs);

    eeIO.updateInputs(eeInputs);
    Logger.processInputs("Arm End Effector", eeInputs);

    shoulderOfflineAlert.set(!shoulderInputs.connected);
    elbowOfflineAlert.set(!elbowInputs.connected);
    eeOfflineAlert.set(!eeInputs.connected);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            shoulderIO.setPdf(
                ArmConfig.shoulderKp.get(),
                ArmConfig.shoulderKd.get(),
                ArmConfig.shoulderKs.get(),
                ArmConfig.shoulderKg.get()),
        ArmConfig.shoulderKp,
        ArmConfig.shoulderKd,
        ArmConfig.shoulderKs,
        ArmConfig.shoulderKg);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            elbowIO.setPdf(
                ArmConfig.elbowKp.get(),
                ArmConfig.elbowKd.get(),
                ArmConfig.elbowKs.get(),
                ArmConfig.elbowKg.get()),
        ArmConfig.elbowKp,
        ArmConfig.elbowKd,
        ArmConfig.elbowKs,
        ArmConfig.elbowKg);

    // Auto-hold algae if present
    if (eeGoal != EndEffectorGoal.CORAL_SCORE
        && eeGoal != EndEffectorGoal.ALGAE_SCORE
        && eeGoal != EndEffectorGoal.EJECT
        && hasAlgae()) {
      setEeGoal(EndEffectorGoal.HOLDING);
    }

    var minSafeGroundIntakeDodgeElevatorHeightMeterVal =
        ArmConfig.minSafeGroundIntakeDodgeElevatorHeightMeter.get();
    var maxUnsafeGroundIntakeDodgeElevatorHeightMeterVal =
        ArmConfig.maxUnsafeGroundIntakeDodgeElevatorHeightMeter.get();

    if (shoulderInputs.positionMeter <= maxUnsafeGroundIntakeDodgeElevatorHeightMeterVal
        || armGoal.getShoulderHeightMeter() <= maxUnsafeGroundIntakeDodgeElevatorHeightMeterVal) {
      needIntakeDodge = true;
    } else if (shoulderInputs.positionMeter >= minSafeGroundIntakeDodgeElevatorHeightMeterVal
        && armGoal.getShoulderHeightMeter() >= minSafeGroundIntakeDodgeElevatorHeightMeterVal) {
      needIntakeDodge = false;
    } else {
      needIntakeDodge =
          RotationUtil.isRotationEnterSpecificAngleArea(
              elbowInputs.positionRad,
              armGoal.getElbowPositionRad(),
              Units.degreesToRadians(170.0),
              Units.degreesToRadians(10.0));
    }
  }

  public void setEeGoal(EndEffectorGoal goal) {
    if (this.eeGoal != goal) {
      this.eeGoal = goal;
      eeIO.setVoltage(eeGoal.getVoltageVolt());
    }
  }

  public boolean hasCoral() {
    return hasCoralSupplier.getAsBoolean();
  }

  public boolean hasAlgae() {
    return hasAlgaeSupplier.getAsBoolean();
  }

  public Command idle() {
    return runOnce(
            () -> {
              setArmGoal(ArmSubsystemGoal.IDLE);
              setEeGoal(EndEffectorGoal.IDLE);
            })
        .withName("Arm/Idle");
  }

  public Command algaeIdle() {
    return runOnce(
            () -> {
              setArmGoal(ArmSubsystemGoal.ALGAE_IDLE);
              setEeGoal(EndEffectorGoal.HOLDING);
            })
        .withName("Arm/AlgaeIdle");
  }

  public Command stop() {
    return runOnce(
            () -> {
              shoulderIO.stop();
              elbowIO.stop();
              eeIO.stop();
            })
        .withName("Arm/Stop");
  }

  public void setArmGoal(ArmSubsystemGoal goal) {
    this.armGoal = goal;

    setElbowPosition(goal.getElbowPositionRad());
    setShoulderPosition(goal.getShoulderHeightMeter());
  }

  public void setShoulderCurrent(double currentAmp) {
    shoulderIO.setCurrent(currentAmp);
  }

  public void setShoulderPosition(double positionMeter) {
    shoulderIO.setPosition(
        positionMeter,
        ArmConfig.shoulderMotionMagicVelMeterPerSec.get(),
        ArmConfig.shoulderMotionMagicAccelMeterPerSec2.get(),
        0.0);
  }

  public void setElbowPosition(double positionRad) {
    elbowIO.setPosition(
        positionRad,
        Units.degreesToRadians(ArmConfig.elbowMotionMagicVelDegreePerSec.get()),
        Units.degreesToRadians(ArmConfig.elbowMotionMagicAccelDegreePerSec2.get()),
        0.0);
  }

  public double getShoulderVelMeterPerSec() {
    return shoulderInputs.velMeterPerSec;
  }

  public void homeShoulder(double positionMeter) {
    shoulderIO.home(positionMeter);
  }

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

  public double getCOGHeightPercent() {
    return MathUtil.clamp(
        shoulderInputs.positionMeter / ArmConfig.SHOULDER_MAX_HEIGHT_METER, 0.0, 1.0);
  }

  public boolean atGoal() {
    return shoulderAtGoal() && elbowAtGoal();
  }

  @AutoLogOutput(key = "Arm/StopAtGoal")
  public boolean stopAtGoal() {
    return shoulderAtGoal()
        && elbowAtGoal()
        && EqualsUtil.epsilonEquals(
            0.0, shoulderInputs.velMeterPerSec, ArmConfig.shoulderStopToleranceMeterPerSec.get())
        && EqualsUtil.epsilonEquals(
            0.0,
            elbowInputs.velRadPerSec,
            Units.degreesToRadians(ArmConfig.elbowStopToleranceDegreePerSec.get()));
  }

  @AutoLogOutput
  public boolean shoulderAtGoal() {
    return shoulderAtPosition(armGoal.getShoulderHeightMeter());
  }

  @AutoLogOutput
  public boolean elbowAtGoal() {
    return elbowAtPosition(armGoal.getElbowPositionRad());
  }

  public boolean shoulderAtPosition(double positionMeter) {
    return EqualsUtil.epsilonEquals(
        positionMeter, shoulderInputs.positionMeter, ArmConfig.shoulderToleranceMeter.get());
  }

  public boolean elbowAtPosition(double positionRad) {
    return EqualsUtil.epsilonEquals(
        positionRad,
        elbowInputs.positionRad,
        Units.degreesToRadians(ArmConfig.elbowToleranceDegree.get()));
  }
}
