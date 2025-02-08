package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.strategies.AutoIntake;
import frc.robot.commands.strategies.AutoScore;
import frc.robot.constants.AutoStrategy;
import frc.robot.constants.AutoStrategyContainer.Action;
import frc.robot.constants.AutoStrategyContainer.ActionType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class StrategyManager {
  public enum AutonomyMode {
    Full,
    Teleop,
    Manual,
  }

  private Queue<Action> actions = null;
  private Command currentCommand = null;
  private Drive drive;
  private ScoringSubsystem scoringSubsysttem;
  private AutonomyMode autonomyMode = AutonomyMode.Full;
  private Action currentAction = null;

  public StrategyManager(Drive drive, ScoringSubsystem scoringSubsystem) {
    actions = new LinkedList<>();
    this.drive = drive;
    this.scoringSubsysttem = scoringSubsystem;
  }

  /**
   * sets the desired level of autonomy
   *
   * @param mode enum representing how autonomous to be
   */
  public void setAutonomyMode(AutonomyMode mode) {
    this.autonomyMode = mode;
  }

  /**
   * gets the current level of autonomy running
   *
   * @return level of autonomy
   */
  public AutonomyMode getAutonomyMode() {
    return autonomyMode;
  }

  /** clears queue */
  public void clearActions() {
    actions.clear();
    if (currentCommand != null) {
      currentCommand.cancel();
    }
  }

  /**
   * adds an Action to the queue to be performed
   *
   * @param action the action to add to queue
   */
  public void addAction(Action action) {
    actions.add(action);
  }

  /**
   * removes the next action in queue
   *
   * @return the next action in queue
   */
  public Action getNextAction() {
    return actions.size() > 0 ? actions.remove() : null;
  }

  /**
   * logs current action queue
   *
   * <p>NOTE: call in periodic
   */
  public void logActions() {
    Logger.recordOutput(
        "StrategyManager/DesiredLocation",
        this.currentAction != null ? this.currentAction.location() : null);
    Logger.recordOutput(
        "StrategyManager/currentAction",
        this.currentAction != null ? this.currentAction.type() : null);
    Logger.recordOutput(
        "StrategyManager/currentCommandStatus",
        this.currentCommand != null ? this.currentCommand.isFinished() : false);
    // Logger.recordOutput(
    //     "StrategyManager/ActionQueue", this.actions.toArray(new Action[this.actions.size()]));
  }

  /**
   * adds actions to queue based on an AutoStrategy
   *
   * @param strategy AutoStrategy defined in java and json
   */
  public void addActionsFromAutoStrategy(AutoStrategy strategy) {
    for (int i = 0; i < strategy.scoringLocations.size(); i++) {
      FieldTarget scoringLevel =
          strategy.scoringLevels.size() > i ? strategy.scoringLevels.get(i) : FieldTarget.L1;
      // add scoring
      this.addAction(
          new Action(
              ActionType.Score, GamePiece.Coral, strategy.scoringLocations.get(i), scoringLevel));

      // intake after each score
      this.addAction(new Action(ActionType.Intake, GamePiece.Coral, strategy.intakeLocation, null));
    }
  }

  /**
   * makes a command based on inputted action
   *
   * @param action action to base command on
   * @return a command to be scheduled
   */
  public Command getCommandFromAction(Action action) {
    if (action == null) {
      return null;
    }
    if (action.type() == ActionType.Intake) {
      switch (this.autonomyMode) {
        case Full:
        case Teleop:
        case Manual:
        default:
          return new AutoIntake(drive, scoringSubsysttem, action.location());
      }
    } else if (action.type() == ActionType.Score) {
      switch (this.autonomyMode) {
        case Full:
        case Teleop:
        case Manual:
        default:
          return new AutoScore(drive, scoringSubsysttem, action.location());
      }
    } else {
      return null;
    }
  }

  public void periodic() {
    if (currentCommand == null || currentCommand.isFinished()) {
      currentAction = getNextAction();
      currentCommand = getCommandFromAction(currentAction);
      System.out.println(currentCommand == null ? "command not found" : "command is found");
      if (currentCommand != null) {
        currentCommand.schedule();
      }
    }

    Logger.recordOutput(
        "StrategyManager/commandScheduled",
        currentCommand != null ? currentCommand.isScheduled() : false);

    Logger.recordOutput("StrategyManager/actionSize", this.actions.size());

    this.logActions();
  }
}
