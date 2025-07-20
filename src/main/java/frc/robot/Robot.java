// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.interfaces.VirtualSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private boolean autonomousHasPrinted;
  private double autonomousStartTime;

  private final RobotContainer robotContainer;

  public Robot() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    RobotController.setBrownoutVoltage(6.0);

    // configure CTRE
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();

    // configure AKit
    loggerInit();

    robotContainer = new RobotContainer();
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  private final void loggerInit() {
    switch (Constants.MODE) {
      case REAL -> {
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        Logger.addDataReceiver(new NT4Publisher());
      }

      case SIM -> Logger.addDataReceiver(new NT4Publisher());

      case REPLAY -> {
        setUseTiming(false); // Run as fast as possible
        var logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }
    Logger.start();

    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              System.out.println(commandPrintHelper(command.getName()));
              logCommandFunction.accept(command, true);
            });

    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              System.out.println(
                  "\u001B[32m" + commandPrintHelper(command.getName()) + "\u001B[0m");
              logCommandFunction.accept(command, false);
            });

    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              System.out.println(
                  "\u001B[31m" + commandPrintHelper(command.getName()) + "\u001B[0m");
              logCommandFunction.accept(command, false);
            });
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);

    if (autonomousCommand != null) {
      if (!autonomousCommand.isScheduled() && !autonomousHasPrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n",
              Timer.getFPGATimestamp() - autonomousStartTime);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n",
              Timer.getFPGATimestamp() - autonomousStartTime);
        }
        autonomousHasPrinted = true;
      }
    }

    VirtualSubsystem.periodicAll();
    CommandScheduler.getInstance().run();

    Threads.setCurrentThreadPriority(true, 10);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousHasPrinted = false;
      autonomousStartTime = Timer.getFPGATimestamp();
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private String commandPrintHelper(String name) {
    switch (name.split("/").length) {
      case 2:
        {
          String subsystem = name.split("/")[0];
          String command = name.split("/")[1];
          StringBuilder sb = new StringBuilder("$ [");
          sb.append(subsystem);
          sb.append("] ");
          sb.append(command);
          return sb.toString();
        }
      case 3:
        {
          String subsystem = name.split("/")[0];
          String command = name.split("/")[1];
          String subcommand = name.split("/")[2];
          StringBuilder sb = new StringBuilder("$ [");
          sb.append(subsystem);
          sb.append("] ");
          sb.append(command);
          sb.append(" => ");
          sb.append(subcommand);
          return sb.toString();
        }
      default:
        return "# " + name;
    }
  }
}
