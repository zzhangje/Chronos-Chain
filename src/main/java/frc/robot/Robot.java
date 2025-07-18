// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.interfaces.VirtualSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);

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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
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
