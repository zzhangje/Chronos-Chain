// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.swerve.Swerve;

public class RobotContainer {
  // subsystem
  private final Swerve swerve;
  private final Intake intake;
  private final Climber climber;

  // states
  boolean g_isClimbing = false;
  boolean s_intakeHasCoral = false;
  boolean s_armHasCoral = false;
  boolean s_armHasAlgae = false;

  public RobotContainer() {
    if (Constants.MODE.equals(Constants.Mode.REAL)) {
      swerve = Swerve.createReal();
      intake = Intake.createReal();
      climber = Climber.createReal();
    } else if (Constants.MODE.equals(Constants.Mode.SIM)) {
      swerve = Swerve.createSim();
      intake = Intake.createSim(() -> s_intakeHasCoral);
      climber = Climber.createSim();
    } else {
      swerve = Swerve.createIO();
      intake = Intake.createIO();
      climber = Climber.createIO();
    }

    configureBindings();
  }

  private void configureBindings() {}

  private void configureSimulation() {}

  private void configureVisualization() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
