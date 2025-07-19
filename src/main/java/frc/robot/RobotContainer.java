// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.math.PoseUtil;
import frc.lib.service.CommandSelector;
import frc.lib.service.GamePieceVisualizer;
import frc.lib.service.Visualizer;
import frc.lib.utils.AllianceFlipUtil;
import frc.reefscape.Field;
import frc.robot.Constants.AscopeAssets;
import frc.robot.Constants.Misc;
import frc.robot.Constants.Ports;
import frc.robot.RobotState.RobotGoal;
import frc.robot.command.UniversalScoreCommand;
import frc.robot.command.general.KsCharacterization;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;
import frc.robot.subsystem.arm.ArmGoal.EndEffectorGoal;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeGoal.IntakePivotGoal;
import frc.robot.subsystem.intake.IntakeGoal.IntakeRollerGoal;
import frc.robot.subsystem.nodeselector.NodeSelector;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.command.TeleopController;
import frc.robot.subsystem.swerve.command.WheelRadiusCharacterization;
import frc.robot.subsystem.swerve.command.WheelRadiusCharacterization.Direction;
import frc.robot.subsystem.vision.ApriltagVision;
import frc.robot.subsystem.vision.ObjectDetectionVision;

public class RobotContainer {
  // driver
  CommandXboxController driver = new CommandXboxController(Ports.Joystick.DRIVER);

  // subsystem
  private final Swerve swerve;
  private final Intake intake;
  private final Climber climber;
  private final Arm arm;

  // service
  private final CommandSelector autoCmdSelector = new CommandSelector("Auto");
  private final NodeSelector nodeSelector = new NodeSelector();

  // states
  boolean g_isClimbing = false;
  boolean s_intakeHasCoral = false;
  boolean s_armHasCoral = true;
  boolean s_armHasAlgae = false;

  public RobotContainer() {
    if (Constants.MODE.equals(Constants.Mode.REAL)) {
      swerve = Swerve.createReal();
      intake = Intake.createReal();
      climber = Climber.createReal();
      arm = Arm.createReal();

      ApriltagVision apriltagVision = ApriltagVision.createReal();
      ObjectDetectionVision objectDetectionVision = ObjectDetectionVision.createReal();
    } else if (Constants.MODE.equals(Constants.Mode.SIM)) {
      swerve = Swerve.createSim();
      intake = Intake.createSim(() -> s_intakeHasCoral);
      climber = Climber.createSim();
      arm = Arm.createSim(() -> s_armHasCoral, () -> s_armHasAlgae);

      GamePieceVisualizer algae =
          new GamePieceVisualizer(
              "Algae",
              1.2,
              1.3,
              PoseUtil.concat(Field.PRESET_ALGAE_POSES, Field.Reef.AlGAE_POSES),
              PoseUtil.concat(Field.Barge.SCORABLE_POSES, Field.Processor.SCORABLE_POSES),
              s_armHasAlgae ? 1 : 0);

      GamePieceVisualizer coral =
          new GamePieceVisualizer(
              "Coral",
              0.5,
              1.0,
              PoseUtil.repeat(Field.PRESET_CORAL_POSES, 5),
              PoseUtil.tolist(Field.Reef.CORAL_POSES),
              (s_armHasCoral ? 1 : 0) + (s_intakeHasCoral ? 1 : 0));

      ApriltagVision apriltagVision =
          ApriltagVision.createSim(() -> RobotState.getOdometry().getEstimatedPose());
      ObjectDetectionVision objectDetectionVision =
          ObjectDetectionVision.createSim(
              () -> RobotState.getOdometry().getEstimatedPose(),
              () -> coral.getPickableGamePiecePose());

      Visualizer visualizer = new Visualizer();
      configureVisualization(visualizer);
      configureSimulation(visualizer, coral, algae);
    } else {
      swerve = Swerve.createIO();
      intake = Intake.createIO();
      climber = Climber.createIO();
      arm = Arm.createIO();
    }

    swerve.setCustomMaxTiltAccelScale(
        () -> Math.pow(1.0 - arm.getCOGHeightPercent() * Misc.accelCOGHeightScaleFactor.get(), 2));
    intake.setNeedDodgeSupplier(() -> arm.isNeedIntakeDodge());

    configureBindings();
    configureAuto();

    System.out.println("##########################################");
    System.out.println("# RobotContainer Initialization Complete #");
    System.out.println("#               Mode: " + Constants.MODE + "               #");
    System.out.println("##########################################");
  }

  private void configureBindings() {
    var teleopDrive =
        new TeleopController(
            swerve,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () ->
                -driver.getRightX()
                    * Math.pow(
                        1.0 - arm.getCOGHeightPercent() * Misc.omegaCOGHeightScaleFactor.get(), 2),
            driver.rightBumper()::getAsBoolean);
    swerve.setDefaultCommand(teleopDrive);

    driver
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getOdometry()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getOdometry().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .ignoringDisable(true));

    driver
        .b()
        .and(() -> !g_isClimbing)
        .whileTrue(
            new UniversalScoreCommand(
                swerve,
                arm,
                intake,
                () ->
                    RobotGoal.scoreProcessor()
                        .setIgnoreArmMoveCondition(nodeSelector.isIgnoreArmMoveCondition())));

    driver
        .x()
        .and(() -> !g_isClimbing)
        .whileTrue(
            new UniversalScoreCommand(
                swerve,
                arm,
                intake,
                () ->
                    RobotGoal.scoreNet()
                        .setIgnoreArmMoveCondition(nodeSelector.isIgnoreArmMoveCondition())));

    driver
        .y()
        .and(() -> !g_isClimbing)
        .whileTrue(new UniversalScoreCommand(swerve, arm, intake, nodeSelector::getSelectedNode));

    driver
        .a()
        .and(() -> !g_isClimbing)
        .onTrue(
            Commands.parallel(
                    arm.idleCommand(),
                    intake.idle(),
                    Commands.runOnce(
                        () -> {
                          Command current = swerve.getCurrentCommand();
                          if (current != null && current != teleopDrive) {
                            current.cancel();
                          }
                        }))
                .withName("Super/Force Idle"));

    driver.leftBumper().onTrue(intake.inject()).onFalse(intake.idle());
  }

  private void configureAuto() {
    new Trigger(() -> DriverStation.isAutonomousEnabled())
        .onTrue(autoCmdSelector.stop())
        .onFalse(autoCmdSelector.run());

    autoCmdSelector.addCommand(
        "Wheel Radius Characterization",
        new WheelRadiusCharacterization(
            swerve,
            () -> RobotState.getOdometry().getEstimatedPose().getRotation(),
            Direction.CLOCKWISE));

    autoCmdSelector.addCommand(
        "Drive Ks Characeterization",
        new KsCharacterization(
            swerve,
            swerve::setModuleDriveCharacterizationCurrent,
            swerve::getModuleDriveCharacterizationVel));
  }

  private void configureSimulation(
      Visualizer visualizer, GamePieceVisualizer coral, GamePieceVisualizer algae) {

    new Trigger(() -> arm.getEeGoal().equals(EndEffectorGoal.CORAL_COLLECT))
        .debounce(0.1)
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      if (intake.getPivotGoal().equals(IntakePivotGoal.REGRASP)
                          && intake.getRollerGoal().equals(IntakeRollerGoal.EJECT)
                          && arm.getArmGoal().equals(ArmSubsystemGoal.CORAL_GROUND_PICK)) {
                        if (s_intakeHasCoral && !s_armHasAlgae) {
                          s_armHasCoral = true;
                          s_intakeHasCoral = false;
                        }
                      }
                      // FIXME: 棒棒糖
                      if (false) {
                        if (!s_armHasAlgae && !s_armHasCoral) {
                          Boolean ret =
                              coral.tryPick(
                                  new Pose3d(RobotState.getOdometry().getEstimatedPose())
                                      .plus(
                                          visualizer
                                              .getComponentTransform(AscopeAssets.ARM)
                                              .plus(Misc.ee_T_coral)));
                          if (ret) {
                            s_armHasCoral = true;
                          }
                        }
                      }
                    })
                .withName("[SIM] Try Regrasp Coral"));

    new Trigger(
            () ->
                intake.getRollerGoal().equals(IntakeRollerGoal.INJECT)
                    && intake.getPivotGoal().equals(IntakePivotGoal.DOWN))
        .debounce(0.5)
        .whileTrue(
            Commands.run(
                    () -> {
                      if (!s_intakeHasCoral) {
                        Boolean ret =
                            coral.tryPick(
                                new Pose3d(RobotState.getOdometry().getEstimatedPose())
                                    .plus(
                                        visualizer
                                            .getComponentTransform(AscopeAssets.INTAKE)
                                            .plus(Misc.intake_T_coral)));
                        if (ret) {
                          s_intakeHasCoral = true;
                        }
                      }
                    })
                .withName("[SIM] Try Pick Coral"));

    new Trigger(
            () ->
                intake.getRollerGoal().equals(IntakeRollerGoal.EJECT)
                    || intake.getRollerGoal().equals(IntakeRollerGoal.TROUGH))
        .debounce(3.5)
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      if (s_intakeHasCoral) {
                        Boolean ret =
                            coral.tryScore(
                                new Pose3d(RobotState.getOdometry().getEstimatedPose())
                                    .plus(
                                        visualizer
                                            .getComponentTransform(AscopeAssets.INTAKE)
                                            .plus(Misc.intake_T_coral)));
                        if (ret) {
                          s_intakeHasCoral = false;
                        } else if (coral.tryEject(
                            new Pose3d(RobotState.getOdometry().getEstimatedPose())
                                .plus(
                                    visualizer
                                        .getComponentTransform(AscopeAssets.INTAKE)
                                        .plus(Misc.intake_T_coral)))) {
                          s_intakeHasCoral = false;
                        }
                      }
                    })
                .withName("[SIM] Try Eject Coral"));

    new Trigger(
            () ->
                arm.getEeGoal().equals(EndEffectorGoal.EJECT)
                    || arm.getEeGoal().equals(EndEffectorGoal.ALGAE_SCORE)
                    || arm.getEeGoal().equals(EndEffectorGoal.CORAL_SCORE))
        .debounce(0.3)
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      if (s_armHasAlgae) {
                        Boolean ret =
                            algae.tryScore(
                                new Pose3d(RobotState.getOdometry().getEstimatedPose())
                                    .plus(
                                        visualizer
                                            .getComponentTransform(AscopeAssets.ARM)
                                            .plus(Misc.ee_T_algae)));
                        if (ret) {
                          s_armHasAlgae = false;
                        } else {
                          if (algae.tryEject(
                              new Pose3d(RobotState.getOdometry().getEstimatedPose())
                                  .plus(
                                      visualizer
                                          .getComponentTransform(AscopeAssets.ARM)
                                          .plus(Misc.ee_T_algae)))) {
                            s_armHasAlgae = false;
                          }
                        }
                      }
                      if (s_armHasCoral) {
                        Boolean ret =
                            coral.tryScore(
                                new Pose3d(RobotState.getOdometry().getEstimatedPose())
                                    .plus(
                                        visualizer
                                            .getComponentTransform(AscopeAssets.ARM)
                                            .plus(Misc.ee_T_coral)));
                        if (ret) {
                          s_armHasCoral = false;
                        } else {
                          if (coral.tryEject(
                              new Pose3d(RobotState.getOdometry().getEstimatedPose())
                                  .plus(
                                      visualizer
                                          .getComponentTransform(AscopeAssets.ARM)
                                          .plus(Misc.ee_T_coral)))) {
                            s_armHasCoral = false;
                          }
                        }
                      }
                    })
                .withName("[SIM] Try Score or Eject"));

    new Trigger(() -> arm.getEeGoal().equals(EndEffectorGoal.ALGAE_COLLECT))
        .debounce(0.2)
        .whileTrue(
            Commands.run(
                    () -> {
                      if (!s_armHasAlgae && !s_armHasCoral) {
                        Boolean ret =
                            algae.tryPick(
                                new Pose3d(RobotState.getOdometry().getEstimatedPose())
                                    .plus(
                                        visualizer
                                            .getComponentTransform(AscopeAssets.INTAKE)
                                            .plus(Misc.ee_T_algae)));
                        if (ret) {
                          s_armHasAlgae = true;
                        }
                      }
                    })
                .withName("[SIM] Try Pick Algae"));
  }

  private void configureVisualization(Visualizer visualizer) {
    visualizer.registerVisualizedComponent(
        Visualizer.BASE_FRAME,
        "chassis",
        AscopeAssets.CHASSIS,
        new Transform3d(
            0.0, 0.0, 0.0, new Rotation3d(Math.PI / 2 * 0, Math.PI / 2 * 2, Math.PI / 2 * 2)));

    // climber
    visualizer.registerVisualizedComponent(
        "chassis", "climber_base", AscopeAssets.CLIMBER_BASE, new Transform3d());
    visualizer.registerVisualizedComponent(
        "climber_base",
        "climber",
        AscopeAssets.CLIMBER,
        () -> new Transform3d(-0.325, 0.0, -0.275, new Rotation3d(0.0, 0.28 - 0.0, 0.0)));

    // intake
    visualizer.registerVisualizedComponent(
        "chassis", "intake_base", AscopeAssets.INTAKE_BASE, new Transform3d());
    visualizer.registerVisualizedComponent(
        "intake_base",
        "intake",
        AscopeAssets.INTAKE,
        () ->
            new Transform3d(
                0.315,
                0.0,
                -0.15,
                new Rotation3d(0.0, Math.PI * 0.72 - intake.getPivotPositionRad(), 0.0)));

    // arm
    visualizer.registerVisualizedComponent(
        "chassis", "elevator_l1", AscopeAssets.ELEVATOR_1, new Transform3d());
    visualizer.registerVisualizedComponent(
        "elevator_l1",
        "elevator_l2",
        AscopeAssets.ELEVATOR_2,
        () -> new Transform3d(0.0, 0.0, -arm.getElevatorHeightMeter(), new Rotation3d()));
    visualizer.registerVisualizedComponent(
        "elevator_l2",
        "carriage",
        AscopeAssets.CARRIAGE,
        () -> new Transform3d(0.0, 0.0, -arm.getCarriageHeightMeter(), new Rotation3d()));
    visualizer.registerVisualizedComponent(
        "carriage",
        "arm",
        AscopeAssets.ARM,
        () ->
            new Transform3d(
                0.0,
                0.0,
                -0.255,
                new Rotation3d(Math.PI / 2 + arm.getElbowPositionRad(), 0.0, 0.0)));

    // game piece
    visualizer.registerVisualizedComponent(
        "intake",
        "coral_intake",
        AscopeAssets.CORAL_INTAKE,
        () ->
            s_intakeHasCoral
                ? Misc.intake_T_coral
                : new Transform3d(0, 0, 0x3f3f3f3f, new Rotation3d()));
    visualizer.registerVisualizedComponent(
        "arm",
        "coral_arm",
        AscopeAssets.CORAL,
        () ->
            s_armHasCoral ? Misc.ee_T_coral : new Transform3d(0, 0, 0x3f3f3f3f, new Rotation3d()));
    visualizer.registerVisualizedComponent(
        "arm",
        "algae_arm",
        AscopeAssets.ALGAE,
        () ->
            s_armHasAlgae ? Misc.ee_T_algae : new Transform3d(0, 0, 0x3f3f3f3f, new Rotation3d()));

    visualizer.print();
  }

  public Command getAutonomousCommand() {
    return autoCmdSelector.getCommand();
  }
}
