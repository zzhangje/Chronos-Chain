package frc.robot.command;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.interfaces.ComposedCommands;
import frc.lib.utils.AllianceFlipUtil;
import frc.reefscape.Field;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmGoal.ArmSubsystemGoal;
import frc.robot.subsystem.arm.command.SetArmGoalCommand;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.command.PounceCoral;
import java.util.Arrays;
import java.util.function.Supplier;

public class PounceCoralCommand extends ComposedCommands {
  public PounceCoralCommand(
      Swerve swerve, Intake intake, Arm arm, Supplier<Translation2d[]> searchAreaVertices) {
    setName("Super/Pounce Coral");
    addRequirements(swerve, intake, arm);

    runningCommand =
        Commands.parallel(
                new PounceCoral(
                        () ->
                            Arrays.stream(searchAreaVertices.get())
                                .map(AllianceFlipUtil::apply)
                                .toArray(Translation2d[]::new),
                        swerve)
                    .andThen(Commands.waitSeconds(1.0)),
                intake.inject(),
                new SetArmGoalCommand(arm, () -> ArmSubsystemGoal.CORAL_GROUND_PICK, () -> false))
            .withDeadline(Commands.waitUntil(intake::hasCoral));
  }

  public static PounceCoralCommand leftStation(Swerve swerve, Intake intake, Arm arm) {
    return new PounceCoralCommand(swerve, intake, arm, () -> Field.LEFT_GROUND_PICK_SEARCH_AREA);
  }

  public static PounceCoralCommand rightStation(Swerve swerve, Intake intake, Arm arm) {
    return new PounceCoralCommand(swerve, intake, arm, () -> Field.RIGHT_GROUND_PICK_SEARCH_AREA);
  }
}
