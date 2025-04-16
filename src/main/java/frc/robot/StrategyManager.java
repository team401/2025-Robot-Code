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
import frc.robot.commands.strategies.AutoDriveToLine;
import frc.robot.commands.strategies.AutoIntake;
import frc.robot.commands.strategies.AutoIntakeBargeAlgae;
import frc.robot.commands.strategies.AutoNetScore;
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
    Smart,
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
  private StringSubscriber coralLevelSelector = table.getStringTopic("coralHeight").subscribe("-1");
  private StringSubscriber algaeScoreLevelSelector =
      table.getStringTopic("algaeScoreHeight").subscribe("-1");
  private StringSubscriber algaeIntakeLevelSelector =
      table.getStringTopic("algaeIntakeHeight").subscribe("-1");
  private StringSubscriber autonomySelector =
      table.getStringTopic("autonomyLevel").subscribe("mid");
  private StringSubscriber gamePieceSelector = table.getStringTopic("gpMode").subscribe("-1");

  private StringPublisher autonomyPublisher = table.getStringTopic("autonomyLevel").publish();
  private DoublePublisher reefLocationPublisher = table.getDoubleTopic("reefTarget").publish();
  private DoublePublisher intakeLocationPublisher = table.getDoubleTopic("stationTarget").publish();
  private StringPublisher coralLevelPublisher = table.getStringTopic("coralHeight").publish();
  private StringPublisher algaeScoreLevelPublisher =
      table.getStringTopic("algaeScoreHeight").publish();
  private StringPublisher algaeIntakeLevelPublisher =
      table.getStringTopic("algaeIntakeHeight").publish();
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
    if (this.autonomyMode != mode) {
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
      case Smart:
        autonomyPublisher.accept("smart");
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
    if (strategy.isBargeAuto) {
      // Barge auto replaces auto with a hardcoded barge auto
      this.addAction(
          new Action(ActionType.Score, GamePiece.Coral, DesiredLocation.Reef0, FieldTarget.L4));
      this.addAction(
          new Action(
              ActionType.DriveToLine, GamePiece.Algae, DesiredLocation.AutoLine, FieldTarget.L2));
      this.addAction(
          new Action(
              ActionType.IntakeAlgae, GamePiece.Algae, DesiredLocation.Algae0, FieldTarget.L2));
      this.addAction(
          new Action(
              ActionType.DriveToLine, GamePiece.Algae, DesiredLocation.NetScore, FieldTarget.L2));
      this.addAction(
          new Action(
              ActionType.NetScore, GamePiece.Algae, DesiredLocation.NetScore, FieldTarget.Net));
      this.addAction( // Drive back to the auto line point so that we're centered on the line before
          // trying to drive into reef
          new Action(
              ActionType.DriveToLine, GamePiece.Algae, DesiredLocation.AutoLine, FieldTarget.L2));
      this.addAction( // Drive into reef to get the move points again
          new Action(
              ActionType.IntakeAlgae, GamePiece.Algae, DesiredLocation.Algae0, FieldTarget.L2));

      return;
    }

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
    } else if (action.type() == ActionType.DriveToLine) {
      return new AutoDriveToLine(drive, action.location());
    } else if (action.type() == ActionType.IntakeAlgae) {
      System.out.println("Generated IntakeAlgae action command with location " + action.location());
      return new AutoIntakeBargeAlgae(drive, scoringSubsystem, DesiredLocation.Algae0);
    } else if (action.type() == ActionType.NetScore) {
      return new AutoNetScore(scoringSubsystem);
    } else {
      return null;
    }
  }

  public void updateScoringLocationsFromSnakeScreen() {
    // drive reef location
    if (drive != null) {
      // Don't set reef target or intake location from SnakeScreen in 'smart' mode, this will be
      // picked automatically
      // based on distance
      if (getAutonomyMode() != AutonomyMode.Smart) {
        drive.updateDesiredLocationFromNetworkTables(
            reefLocationSelector.get(), gamePieceSelector.get().equalsIgnoreCase("algae"));

        // 20: left; 21: right
        if (gamePieceSelector.get().equalsIgnoreCase("coral")) {
          drive.setDesiredIntakeLocation(
              intakeLocationSelector.get() == 20
                  ? DesiredLocation.CoralStationLeft
                  : DesiredLocation.CoralStationRight);
        }
      }
    }

    // scoring level selection
    if (scoringSubsystem != null) {
      // update scoring gamepiece
      String gamePiece = gamePieceSelector.get();
      // System.out.println(gamePiece);

      if (gamePiece.equalsIgnoreCase("coral")) {
        scoringSubsystem.setGamePiece(GamePiece.Coral);
      } else if (gamePiece.equalsIgnoreCase("algae")) {
        // System.out.println("Setting algae in strategymanager");
        scoringSubsystem.setGamePiece(GamePiece.Algae);
      }

      // Don't automatically set level in smart mode FOR ALGAE; this will be set when the warmup
      // trigger is
      // pressed. This can't happen in periodic because algae level is automatically determined when
      // intake is pressed and would be overridden by this in each loop.
      if (getAutonomyMode() != AutonomyMode.Smart
          || scoringSubsystem.getGamePiece() != GamePiece.Algae) {
        updateScoringLevelFromNetworkTables();
      }
    }

    // update autonomy level
    String autonomyLevel = autonomySelector.get();

    if (autonomyLevel.equalsIgnoreCase("high")) {
      this.setAutonomyMode(AutonomyMode.Full);
    } else if (autonomyLevel.equalsIgnoreCase("smart")) {
      this.setAutonomyMode(AutonomyMode.Smart);
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
    if (!DriverStation.isAutonomous()) {
      this.updateScoringLocationsFromSnakeScreen();
    }
    this.publishCoralAndAlgae();
  }

  public void updateScoringLevelFromNetworkTables() {
    scoringSubsystem.updateScoringLevelFromNetworkTables(
        coralLevelSelector.get(), algaeIntakeLevelSelector.get(), algaeScoreLevelSelector.get());
  }

  public void publishDefaultSubsystemValues() {
    if (DriverStation.isTeleop()) {
      this.setAutonomyMode(AutonomyMode.Smart);
    }
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

      scoringSubsystem.setTarget(FieldTarget.L4);

      // publish default level
      switch (scoringSubsystem.getCoralTarget()) {
        case L1:
          coralLevelPublisher.accept("level1");
          break;
        case L2:
          coralLevelPublisher.accept("level2");
          break;
        case L3:
          coralLevelPublisher.accept("level3");
          break;
        case L4:
          coralLevelPublisher.accept("level4");
          break;
        default:
          break;
      }

      switch (scoringSubsystem.getAlgaeIntakeTarget()) {
        case L2:
          algaeIntakeLevelPublisher.accept("level2");
          break;
        case L3:
          algaeIntakeLevelPublisher.accept("level3");
          break;
        default:
          break;
      }

      switch (scoringSubsystem.getAlgaeScoreTarget()) {
        case Processor:
          algaeScoreLevelPublisher.accept("level1");
          break;
        case Net:
          algaeScoreLevelPublisher.accept("level4");
          break;
        default:
          break;
      }
    }

    if (drive != null) {
      // publish default reef location
      int reefLocation = drive.getDesiredLocationIndex();
      if (gamePieceSelector.get().equalsIgnoreCase("algae")) {
        reefLocation = drive.getDesiredAlgaeLocationIndex();
      }

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
      case Smart:
        autonomyPublisher.accept("smart");
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

    this.currentCommand = null;
    this.currentAction = null;

    actions.clear();
    this.addActionsFromAutoStrategy(strategy);
    System.out.println("New actions loaded: " + String.valueOf(actions.size()));

    this.publishDefaultSubsystemValues();
  }

  /**
   * call in robot container teleop init to clear actions from auto and publish default snakescreen
   * values
   */
  public void teleopInit() {
    this.setAutonomyMode(AutonomyMode.Smart);

    this.clearActions();

    this.currentCommand = null;
    this.currentAction = null;

    this.publishDefaultSubsystemValues();
  }
}
