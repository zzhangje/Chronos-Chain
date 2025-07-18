package frc.lib.service;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.dashboard.SwitchableChooser;
import frc.lib.interfaces.VirtualSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * A subsystem that allows selecting and configuring Commands via the SmartDashboard. Each command
 * can define up to a fixed number of configuration questions to customize its behavior.
 */
public class CommandSelector extends VirtualSubsystem {
  private static final int MAX_QUESTIONS = 2;
  private static final SelectedCommand DEFAULT_COMMAND =
      new SelectedCommand("None", List.of(), rs -> Commands.none());

  /**
   * Represents a selectable command that may be configured with user input.
   *
   * @param name Name shown on the dashboard
   * @param questions List of configuration questions for this command
   * @param commandBuilder Function to build the command using the user's responses
   */
  private record SelectedCommand(
      String name,
      List<CommandQuestion> questions,
      Function<List<String>, Command> commandBuilder) {}

  /**
   * Represents a single configuration question for a command.
   *
   * @param question The question text shown on the dashboard
   * @param responses List of possible answers for the user to choose from
   */
  public record CommandQuestion(String question, List<String> responses) {}

  private final LoggedDashboardChooser<SelectedCommand> commandChooser;
  private final List<StringPublisher> questionPublishers;
  private final List<SwitchableChooser> questionChoosers;

  private SelectedCommand lastCommand = DEFAULT_COMMAND;
  private List<String> lastResponses = List.of();

  /**
   * Constructs a CommandSelector with a specified key for dashboard display.
   *
   * @param key The key used to identify this selector on the dashboard
   */
  public CommandSelector(String key) {
    // Initialize the command selector on SmartDashboard
    commandChooser = new LoggedDashboardChooser<>(key + "/Command");
    commandChooser.addDefaultOption(DEFAULT_COMMAND.name, DEFAULT_COMMAND);

    // Initialize publishers and choosers for each question slot
    questionPublishers = new ArrayList<>();
    questionChoosers = new ArrayList<>();
    for (int i = 0; i < MAX_QUESTIONS; i++) {
      String questionKey = key + "/Question #" + (i + 1);
      StringPublisher publisher =
          NetworkTableInstance.getDefault()
              .getStringTopic("/SmartDashboard/" + questionKey)
              .publish();
      publisher.set("NA");
      questionPublishers.add(publisher);
      questionChoosers.add(new SwitchableChooser(questionKey));
    }
  }

  @Override
  public void periodic() {
    SelectedCommand selectedCommand = commandChooser.get();
    if (selectedCommand == null) {
      return;
    }

    // If the selected command has changed, update the UI with its configuration questions
    if (!selectedCommand.equals(lastCommand)) {
      System.out.println("Command switched -> " + selectedCommand.name());
      List<CommandQuestion> questions = selectedCommand.questions();

      for (int i = 0; i < MAX_QUESTIONS; i++) {
        if (i < questions.size()) {
          CommandQuestion question = questions.get(i);
          questionPublishers.get(i).set(question.question());
          questionChoosers.get(i).setOptions(question.responses().toArray(String[]::new));
        } else {
          questionPublishers.get(i).set("");
          questionChoosers.get(i).setOptions(new String[] {});
        }
      }
    }

    // Collect current user responses for the selected command's questions
    List<String> responses = new ArrayList<>();
    for (int i = 0; i < selectedCommand.questions().size(); i++) {
      String response = questionChoosers.get(i).get();
      responses.add(
          response == null ? selectedCommand.questions().get(i).responses().get(0) : response);
    }

    // Save current state for next iteration
    lastCommand = selectedCommand;
    lastResponses = responses;
  }

  /**
   * Adds a command with no configuration questions.
   *
   * @param name Name of the command shown on the dashboard
   * @param command The command to run
   */
  public void addCommand(String name, Command command) {
    addCommand(name, rs -> command);
  }

  /**
   * Adds a command that dynamically builds a command based on user input.
   *
   * @param name Name of the command shown on the dashboard
   * @param commandBuilder Function that builds the command using the user's responses
   */
  public void addCommand(String name, Function<List<String>, Command> commandBuilder) {
    addCommand(name, List.of(), commandBuilder);
  }

  /**
   * Adds a configurable command with setup questions shown on the dashboard.
   *
   * @param name Name of the command shown on the dashboard
   * @param questions List of configuration questions for this command
   * @param commandBuilder Function that builds the command using the user's responses
   */
  public void addCommand(
      String name,
      List<CommandQuestion> questions,
      Function<List<String>, Command> commandBuilder) {
    commandChooser.addOption(name, new SelectedCommand(name, questions, commandBuilder));
  }

  /**
   * Gets the currently configured command based on the selected option and user input.
   *
   * @return The command representing the selected configuration
   */
  public Command getCommand() {
    System.out.println("Command built: " + lastCommand.name());
    return lastCommand.commandBuilder().apply(lastResponses);
  }
}
