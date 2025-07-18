package frc.robot.subsystem.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.dashboard.Alert;
import frc.lib.interfaces.motor.GenericArmIO;
import frc.lib.interfaces.motor.GenericArmIOInputsAutoLogged;
import frc.lib.interfaces.motor.GenericArmIOKraken;
import frc.lib.interfaces.motor.GenericArmIOSim;
import frc.lib.interfaces.motor.GenericRollerIO;
import frc.lib.interfaces.motor.GenericRollerIOInputsAutoLogged;
import frc.lib.interfaces.motor.GenericRollerIOKraken;
import frc.lib.interfaces.motor.GenericRollerIOSim;
import frc.robot.Constants.Ports;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final GenericArmIO pullIO;
  private final GenericArmIOInputsAutoLogged pullInputs = new GenericArmIOInputsAutoLogged();
  private final GenericRollerIO lickIO;
  private final GenericRollerIOInputsAutoLogged lickInputs = new GenericRollerIOInputsAutoLogged();
  private final Alert pullOfflineAlert =
      new Alert("Climber pull motor offline!", Alert.AlertType.WARNING);
  private final Alert lickOfflineAlert =
      new Alert("Climber lick motor offline!", Alert.AlertType.WARNING);

  private Climber(GenericArmIO pullIO, GenericRollerIO lickIO) {
    this.pullIO = pullIO;
    this.lickIO = lickIO;
  }

  public Command idle() {
    Command cmd =
        new InstantCommand(
                () -> {
                  pullIO.setVoltage(ClimberConfig.pullVoltage.get());
                })
            .andThen(
                new WaitUntilCommand(() -> atTargetPosition(ClimberConfig.HOME_POSITION_DEGREE)))
            .finallyDo((interrupted) -> stop())
            .withName("Climber/Idle");
    cmd.addRequirements(this);
    return cmd;
  }

  public Command ready() {
    Command cmd =
        new InstantCommand(
                () -> {
                  pullIO.setVoltage(ClimberConfig.pullVoltage.get());
                  lickIO.setVoltage(ClimberConfig.lickVoltage.get());
                })
            .andThen(
                new WaitUntilCommand(() -> atTargetPosition(ClimberConfig.readyPositionDeg.get())))
            .finallyDo((interrupted) -> stop())
            .withName("Climber/Ready");
    cmd.addRequirements(this);
    return cmd;
  }

  public Command pull() {
    Command cmd =
        new InstantCommand(
                () -> {
                  pullIO.setVoltage(ClimberConfig.pullVoltage.get());
                  lickIO.stop();
                })
            .andThen(
                new WaitUntilCommand(() -> atTargetPosition(ClimberConfig.pullPositionDeg.get())))
            .finallyDo((interrupted) -> stop())
            .withName("Climber/Pull");
    cmd.addRequirements(this);
    return cmd;
  }

  public Command stop() {
    return new InstantCommand(
            () -> {
              pullIO.stop();
              lickIO.stop();
            },
            this)
        .withName("Climber/Stop");
  }

  public double getPullPositionRad() {
    return pullInputs.positionRad;
  }

  @AutoLogOutput
  private boolean atTargetPosition(double targetPositionRad) {
    return pullInputs.positionRad >= targetPositionRad;
  }

  @Override
  public void periodic() {
    pullIO.updateInputs(pullInputs);
    Logger.processInputs("Climber/Pull", pullInputs);
    pullOfflineAlert.set(!pullInputs.connected);

    lickIO.updateInputs(lickInputs);
    Logger.processInputs("Climber/Lick", lickInputs);
    lickOfflineAlert.set(!lickInputs.connected);
  }

  public Climber createSim() {
    return new Climber(
        new GenericArmIOSim(
            DCMotor.getKrakenX60Foc(1),
            ClimberConfig.REDUCTION,
            0.46,
            0.35,
            Units.degreesToRadians(Double.NEGATIVE_INFINITY),
            Units.degreesToRadians(Double.POSITIVE_INFINITY),
            Units.degreesToRadians(ClimberConfig.HOME_POSITION_DEGREE)),
        new GenericRollerIOSim(DCMotor.getFalcon500(1), 1, 1));
  }

  public Climber createReal() {
    return new Climber(
        new GenericArmIOKraken(
            "Climber/Pull",
            Ports.Can.CLIMBER_ARM,
            ClimberConfig.getPullTalonConfig(),
            ClimberConfig.HOME_POSITION_DEGREE),
        new GenericRollerIOKraken(
            "Climber/Lick", Ports.Can.CLIMBER_ROLLER, ClimberConfig.getLickTalonConfig()));
  }

  public static Climber createIO() {
    return new Climber(new GenericArmIO() {}, new GenericRollerIO() {});
  }
}
