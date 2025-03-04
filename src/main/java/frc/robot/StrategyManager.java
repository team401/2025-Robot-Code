package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.strategies.AutoIntake;
import frc.robot.commands.strategies.AutoScore;
import frc.robot.constants.AutoStrategy;
import frc.robot.constants.AutoStrategyContainer.Action;
import frc.robot.constants.AutoStrategyContainer.ActionType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class StrategyManager {
  public enum AutonomyMode {
    Full,
    Mixed,
    Manual,
  }

  public static StrategyManager instance;

  private Queue<Action> actions = null;
  private Command currentCommand = null;
  private Drive drive;
  private ScoringSubsystem scoringSubsystem;
  private AutonomyMode autonomyMode = AutonomyMode.Full;
  private Action currentAction = null;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("");
  private DoubleSubscriber reefLocationSelector = table.getDoubleTopic("reefTarget").subscribe(-1);
  private DoubleSubscriber intakeLocationSelector =
      table.getDoubleTopic("stationTarget").subscribe(-1);
  private StringSubscriber reefLevelSelector = table.getStringTopic("scoreHeight").subscribe("-1");
  private StringSubscriber autonomySelector =
      table.getStringTopic("autonomyLevel").subscribe("mid");
  private StringSubscriber gamePieceSelector = table.getStringTopic("gpMode").subscribe("-1");

  private StringPublisher autonomyPublisher = table.getStringTopic("autonomyLevel").publish();
  private DoublePublisher reefLocationPublisher = table.getDoubleTopic("reefTarget").publish();
  private DoublePublisher intakeLocationPublisher = table.getDoubleTopic("stationTarget").publish();
  private StringPublisher reefLevelPublisher = table.getStringTopic("scoreHeight").publish();
  private StringPublisher gamePiecePublisher = table.getStringTopic("gpMode").publish();
  private BooleanPublisher hasCoralPublisher = table.getBooleanTopic("hasCoral").publish();
  private BooleanPublisher hasAlgaePublisher = table.getBooleanTopic("hasAlgae").publish();

  public StrategyManager(Drive drive, ScoringSubsystem scoringSubsystem) {
    actions = new LinkedList<>();
    this.drive = drive;
    this.scoringSubsystem = scoringSubsystem;

    if (instance == null) {
      instance = this;
    } else {
      System.out.println("Instantiated strategy manager twice :(");
    }
  }

  public static StrategyManager getInstance() {
    return instance;
  }

  /**
   * sets the desired level of autonomy
   *
   * @param mode enum representing how autonomous to be
   */
  public void setAutonomyMode(AutonomyMode mode) {
    if (this.autonomyMode == mode) {
      this.autonomyMode = mode;
      onAutonomyModeChange(mode);
    }
    this.autonomyMode = mode;
  }

  /**
   * sets the desired level of autonomy and publishes it to snakescreen
   *
   * @param mode enum representing how autonomous to be
   */
  public void setAutonomyModeAndPublish(AutonomyMode mode) {
    setAutonomyMode(mode);

    switch (mode) {
      case Full:
        autonomyPublisher.accept("high");
        break;
      case Mixed:
        autonomyPublisher.accept("mid");
        break;
      case Manual:
        autonomyPublisher.accept("low");
        break;
    }
  }

  public void onAutonomyModeChange(AutonomyMode newMode) {}

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
    Logger.recordOutput("StrategyManager/commandNull", currentCommand == null);
    Logger.recordOutput(
        "StrategyManager/commandScheduled",
        currentCommand != null ? currentCommand.isScheduled() : false);

    Logger.recordOutput("StrategyManager/actionSize", this.actions.size());
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

      if (!(i == strategy.scoringLocations.size() - 1 && !strategy.intakeAfterLastScore)) {
        // intake after each score, only if it is not the last scoring location and the auto isn't
        // configured to not intake after last score
        this.addAction(
            new Action(ActionType.Intake, GamePiece.Coral, strategy.intakeLocation, null));
      }
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
        case Mixed:
        case Manual:
        default:
          return new AutoIntake(drive, scoringSubsystem, action.location(), action.scoringTarget());
      }
    } else if (action.type() == ActionType.Score) {
      switch (this.autonomyMode) {
        case Full:
        case Mixed:
        case Manual:
        default:
          return new AutoScore(drive, scoringSubsystem, action.location(), action.scoringTarget());
      }
    } else {
      return null;
    }
  }

  public void updateScoringLocationsFromSnakeScreen() {
    // drive reef location
    if (drive != null) {
      drive.updateDesiredLocationFromNetworkTables(
          reefLocationSelector.get(), gamePieceSelector.get().equalsIgnoreCase("algae"));

      // 20: left; 21: right
      drive.setDesiredIntakeLocation(
          intakeLocationSelector.get() == 20
              ? DesiredLocation.CoralStationLeft
              : DesiredLocation.CoralStationRight);
    }

    // scoring level selection
    if (scoringSubsystem != null) {
      scoringSubsystem.updateScoringLevelFromNetworkTables(reefLevelSelector.get());

      // update scoring gamepiece
      String gamePiece = gamePieceSelector.get();

      if (gamePiece.equalsIgnoreCase("coral")) {
        scoringSubsystem.setGamePiece(GamePiece.Coral);
      } else if (gamePiece.equalsIgnoreCase("algae")) {
        scoringSubsystem.setGamePiece(GamePiece.Algae);
      }
    }

    // update autonomy level
    String autonomyLevel = autonomySelector.get();

    if (autonomyLevel.equalsIgnoreCase("high")) {
      this.setAutonomyMode(AutonomyMode.Full);
    } else if (autonomyLevel.equalsIgnoreCase("mid")) {
      this.setAutonomyMode(AutonomyMode.Mixed);
    } else if (autonomyLevel.equalsIgnoreCase("low")) {
      this.setAutonomyMode(AutonomyMode.Manual);
    }
  }

  public void publishCoralAndAlgae() {
    if (scoringSubsystem != null) {
      hasCoralPublisher.accept(scoringSubsystem.isCoralDetected());
      hasAlgaePublisher.accept(scoringSubsystem.isAlgaeDetected());
    }
  }

  public void periodic() {
    if (currentCommand == null || currentCommand.isFinished()) {
      if (currentCommand != null) {
        System.out.println(currentCommand.getName() + " was finished.");
      }
      currentAction = getNextAction();
      currentCommand = getCommandFromAction(currentAction);
      if (currentCommand != null) {
        CommandScheduler.getInstance().schedule(currentCommand);
        System.out.println("Scheduled new command");
      }
    }

    // updates snakescreen with current locations to watch auto run
    if (DriverStation.isAutonomousEnabled()) {
      this.publishDefaultSubsystemValues();
    }

    this.logActions();

    // send and receive from SnakeScreen
    this.updateScoringLocationsFromSnakeScreen();
    this.publishCoralAndAlgae();
  }

  public void publishDefaultSubsystemValues() {
    if (scoringSubsystem != null) {
      // publish default game piece
      switch (scoringSubsystem.getGamePiece()) {
        case Coral:
          gamePiecePublisher.accept("coral");
          break;
        case Algae:
          gamePiecePublisher.accept("algae");
          break;
        default:
          break;
      }

      // publish default level
      switch (scoringSubsystem.getTarget()) {
        case L1:
          reefLevelPublisher.accept("level1");
          break;
        case L2:
          reefLevelPublisher.accept("level2");
          break;
        case L3:
          reefLevelPublisher.accept("level3");
          break;
        case L4:
          reefLevelPublisher.accept("level4");
          break;
        default:
          break;
      }
    }

    if (drive != null) {
      // publish default reef location
      int reefLocation = drive.getDesiredLocationIndex();

      if (reefLocation != -1) {
        reefLocationPublisher.accept(reefLocation);
      }

      // publish default intake location
      // 20: left; 21: right
      double intakeLocation =
          drive.getDesiredIntakeLocation() == DesiredLocation.CoralStationLeft ? 20 : 21;
      intakeLocationPublisher.accept(intakeLocation);
    }

    // publish autonomy mode
    switch (this.getAutonomyMode()) {
      case Full:
        autonomyPublisher.accept("high");
        break;
      case Mixed:
        autonomyPublisher.accept("mid");
        break;
      case Manual:
        autonomyPublisher.accept("low");
        break;
      default:
        break;
    }
  }

  /**
   * call in robot container autonomous init to schedule actions and publish to snakescreen
   *
   * @param strategy the auto actions to run
   */
  public void autonomousInit(AutoStrategy strategy) {
    this.setAutonomyMode(AutonomyMode.Full);

    this.addActionsFromAutoStrategy(strategy);

    this.publishDefaultSubsystemValues();
  }

  /**
   * call in robot container teleop init to clear actions from auto and publish default snakescreen
   * values
   */
  public void teleopInit() {
    this.setAutonomyMode(AutonomyMode.Mixed);

    this.clearActions();

    this.publishDefaultSubsystemValues();
  }
}
