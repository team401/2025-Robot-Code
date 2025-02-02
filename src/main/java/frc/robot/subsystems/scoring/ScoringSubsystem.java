package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExampleElevatorCommand;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.states.IdleState;
import frc.robot.subsystems.scoring.states.IntakeState;
import frc.robot.subsystems.scoring.states.ScoreState;
import frc.robot.subsystems.scoring.states.WarmupState;
import org.littletonrobotics.junction.Logger;

public class ScoringSubsystem extends SubsystemBase {
  private ElevatorMechanism elevatorMechanism;
  private ClawMechanism clawMechanism;

  // Keep track of an instance to pass to state machine
  private static ScoringSubsystem instance;

  /**
   * Whether or not we should automatically transition through the workflow Idle -> Intake -> Warmup
   * -> Score
   */
  private boolean autoTransition = true;

  /**
   * This boolean exists to temporarily stop the state machine from running
   *
   * <p>It is intended to be used for test modes, e.g. ClawOvershootTuning needs to be able to
   * manually eject an object without forcing the state machine to agree.
   */
  private boolean overrideStateMachine = false;

  public enum FieldTarget {
    L1,
    L2,
    L3,
    L4,
    Net,
    Processor,
    Ground
  }

  /**
   * Keep track of which FieldTarget we're aiming for.
   *
   * <p>Note, this also includes which level to intake algae from if gamePiece is algae. Coral is
   * not included, since we always intake it from coral station.
   */
  private FieldTarget currentTarget = FieldTarget.L1;

  public enum GamePiece {
    Coral,
    Algae
  }

  private GamePiece currentPiece = GamePiece.Coral;

  private enum ScoringState implements StateContainer {
    Idle(new IdleState(instance)),
    Intake(new IntakeState(instance)),
    Warmup(new WarmupState(instance)),
    Score(new ScoreState(instance));

    private final PeriodicStateInterface state;

    ScoringState(PeriodicStateInterface state) {
      this.state = state;
    }

    @Override
    public PeriodicStateInterface getState() {
      return state;
    }
  }

  public enum ScoringTrigger {
    BeginIntake,
    DoneIntaking,
    CancelAction,
    ToggleWarmup, // warmup button toggles warmup <-> idle
    ScoredPiece,
    ReturnToIdle, // Return to idle when a warmup/score state no longer detects a gamepiece
  }

  private StateMachineConfiguration<ScoringState, ScoringTrigger> stateMachineConfiguration;

  private StateMachine<ScoringState, ScoringTrigger> stateMachine;

  public ScoringSubsystem(ElevatorMechanism elevatorMechanism, ClawMechanism clawMechanism) {
    this.elevatorMechanism = elevatorMechanism;
    this.clawMechanism = clawMechanism;

    setDefaultCommand(new ExampleElevatorCommand(this));

    instance = this;

    stateMachineConfiguration = new StateMachineConfiguration<>();

    stateMachineConfiguration
        .configure(ScoringState.Idle)
        .permitIf(
            ScoringTrigger.BeginIntake,
            ScoringState.Intake,
            () -> !(isCoralDetected() || isAlgaeDetected()))
        .permit(ScoringTrigger.ToggleWarmup, ScoringState.Warmup);

    stateMachineConfiguration
        .configure(ScoringState.Intake)
        // .configureOnEntryAction(ScoringState.Intake.state::onEntry) // Just tried this line too
        // but it didn't work
        // If autoTransition, go straight to warmup from intake once we're done
        // Otherwise, return to idle
        .permitIf(ScoringTrigger.DoneIntaking, ScoringState.Warmup, () -> autoTransition)
        .permitIf(ScoringTrigger.DoneIntaking, ScoringState.Idle, () -> !autoTransition)
        .permit(ScoringTrigger.CancelAction, ScoringState.Idle);

    stateMachineConfiguration
        .configure(ScoringState.Warmup)
        .permit(ScoringTrigger.ToggleWarmup, ScoringState.Idle)
        .permit(ScoringTrigger.ReturnToIdle, ScoringState.Idle);

    stateMachineConfiguration
        .configure(ScoringState.Score)
        .permit(ScoringTrigger.ScoredPiece, ScoringState.Idle)
        .permit(ScoringTrigger.ReturnToIdle, ScoringState.Idle);

    stateMachine = new StateMachine<>(stateMachineConfiguration, ScoringState.Idle);

    SmartDashboard.putBoolean("scoring/fireStartIntaking", false);
  }

  /**
   * Fire a trigger on the Scoring state machine
   *
   * @param trigger The trigger to fire.
   */
  public void fireTrigger(ScoringTrigger trigger) {
    stateMachine.fire(trigger);
  }

  /**
   * Send a new goal height to the elevator mechanism
   *
   * @param goalHeight Goal height to command elevator mechanism to
   */
  public void setElevatorGoalHeight(Distance goalHeight) {
    if (JsonConstants.scoringFeatureFlags.runElevator) {
      elevatorMechanism.setGoalHeight(goalHeight);
    }
  }

  /**
   * Get the current height of the elevator.
   *
   * <p>This height is determined by the {@link ElevatorMechanism}.
   *
   * @return
   */
  public Distance getElevatorHeight() {
    if (JsonConstants.scoringFeatureFlags.runElevator) {
      return elevatorMechanism.getElevatorHeight();
    } else {
      return Meters.zero();
    }
  }

  /**
   * Run the claw rollers at a given voltage.
   *
   * <p>Positive voltage is the intake/score direction.
   *
   * @param voltage The voltage to run the claw at.
   */
  public void setClawRollerVoltage(Voltage voltage) {
    if (JsonConstants.scoringFeatureFlags.runClaw) {
      clawMechanism.getIO().setVoltage(voltage);
    }
  }

  public Angle getClawRollerPosition() {
    if (JsonConstants.scoringFeatureFlags.runClaw) {
      return clawMechanism.getIO().getClawMotorPos();
    }
    return Rotations.zero();
  }

  /**
   * Check whether the claw mechanism currently detects a coral.
   *
   * @return True if yes, false if no.
   */
  public boolean isCoralDetected() {
    if (JsonConstants.scoringFeatureFlags.runClaw) {
      return clawMechanism.getIO().isCoralDetected();
    } else {
      return false;
    }
  }

  /**
   * Check whether the claw mechanism currently detects an algae.
   *
   * @return True if yes, false if no.
   */
  public boolean isAlgaeDetected() {
    if (JsonConstants.scoringFeatureFlags.runClaw) {
      return clawMechanism.getIO().isAlgaeDetected();
    } else {
      return false;
    }
  }

  /**
   * Set the field target of the scoring subsystem.
   *
   * @param target The field target that scoring will aim for, e.g. L2
   */
  public void setTarget(FieldTarget target) {
    currentTarget = target;
  }

  /**
   * Get the current field target of the scoring subsystem.
   *
   * @return Which field target scoring is currently trying to score in. E.g. L2
   */
  public FieldTarget getTarget() {
    return currentTarget;
  }

  /**
   * Set which game piece the scoring subsystem should try to intake/score.
   *
   * @param piece The piece type
   */
  public void setGamePiece(GamePiece piece) {
    currentPiece = piece;
  }

  /**
   * Get the current game piece target of the scoring subsystem
   *
   * @return Whichever piece Scoring has been commanded to handle most recently
   */
  public GamePiece getGamePiece() {
    return currentPiece;
  }

  /**
   * Set whether or not to temporarily override/disable the state machine. This should only be used
   * by test modes!
   *
   * @param doOverrideStateMachine True if the state machine is disabled, and false if the state
   *     machine should run. This value is false on initialization and will only change when this
   *     method is called.
   */
  public void setOverrideStateMachine(boolean doOverrideStateMachine) {
    overrideStateMachine = doOverrideStateMachine;
  }

  @Override
  public void periodic() {
    boolean fireStartIntaking = SmartDashboard.getBoolean("scoring/fireStartIntaking", false);
    if (fireStartIntaking) {
      setGamePiece(GamePiece.Algae);
      setTarget(FieldTarget.L2);
      stateMachine.fire(ScoringTrigger.BeginIntake);
      if (!stateMachine.getTransitionInfo().wasFail()) {
        System.out.println(stateMachine.getTransitionInfo().getTransition().isInternal());
      }
    }

    if (!overrideStateMachine) {
      stateMachine.periodic();
    }

    if (JsonConstants.scoringFeatureFlags.runElevator) {
      elevatorMechanism.periodic();
    }

    if (JsonConstants.scoringFeatureFlags.runClaw) {
      clawMechanism.periodic();
    }

    Logger.recordOutput("scoring/state", stateMachine.getCurrentState());
  }

  /** This method must be called by RobotContainer, as it does not run automatically! */
  public void testPeriodic() {
    if (JsonConstants.scoringFeatureFlags.runElevator) {
      elevatorMechanism.testPeriodic();
    }

    if (JsonConstants.scoringFeatureFlags.runClaw) {
      clawMechanism.testPeriodic();
    }
  }
}
