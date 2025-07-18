package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.dashboard.LoggedTunableNumber;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class KsCharacterization extends Command {
  private static final LoggedTunableNumber currentRampFactor =
      new LoggedTunableNumber("Auto/KsCharacterization/CurrentRampPerSec", 1.0);
  private static final LoggedTunableNumber minVelocity =
      new LoggedTunableNumber("Auto/KsCharacterization/MinStaticVelocity", 0.1);

  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;

  public KsCharacterization(
      Subsystem subsystem, DoubleConsumer currentConsumer, DoubleSupplier velocitySupplier) {
    inputConsumer = currentConsumer;
    this.velocitySupplier = velocitySupplier;
    addRequirements(subsystem);
    withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    setName("[Auto] Ks Characterization");
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    currentInput = timer.get() * currentRampFactor.get();
    inputConsumer.accept(currentInput);
  }

  @Override
  public boolean isFinished() {
    return velocitySupplier.getAsDouble() >= minVelocity.get();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Calculated Ks: " + currentInput + " amps");
    inputConsumer.accept(0);
  }
}
