package frc.robot.subsystems.scoring;

import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExampleElevatorCommand;
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
        .permit(ScoringTrigger.BeginIntake, ScoringState.Intake);

    stateMachineConfiguration
        .configure(ScoringState.Intake)
        // .permitIf(ScoringStateMachineTriggers.BeginIntake, ScoringStateContainer.Idle, () ->
        // isCoralDetected())
        .permit(ScoringTrigger.DoneIntaking, ScoringState.Idle)
        .permit(ScoringTrigger.CancelAction, ScoringState.Idle);

    stateMachineConfiguration
        .configure(ScoringState.Warmup)
        .permit(ScoringTrigger.CancelAction, ScoringState.Idle);

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
    elevatorMechanism.setGoalHeight(goalHeight);
  }

  /**
   * Get the current height of the elevator.
   *
   * <p>This height is determined by the {@link ElevatorMechanism}.
   *
   * @return
   */
  public Distance getElevatorHeight() {
    return elevatorMechanism.getElevatorHeight();
  }

  /**
   * Run the claw rollers at a given voltage.
   *
   * <p>Positive voltage is the intake/score direction.
   *
   * @param voltage The voltage to run the claw at.
   */
  public void setClawRollerVoltage(Voltage voltage) {
    clawMechanism.getIO().setVoltage(voltage);
  }

  /**
   * Check whether the claw mechanism currently detects a coral.
   *
   * @return True if yes, false if no.
   */
  public boolean isCoralDetected() {
    return clawMechanism.getIO().isCoralDetected();
  }

  /**
   * Check whether the claw mechanism currently detects an algae.
   *
   * @return True if yes, false if no.
   */
  public boolean isAlgaeDetected() {
    return clawMechanism.getIO().isAlgaeDetected();
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

  @Override
  public void periodic() {
    boolean fireStartIntaking = SmartDashboard.getBoolean("scoring/fireStartIntaking", false);
    if (fireStartIntaking) {
      stateMachine.fire(ScoringTrigger.BeginIntake);
    }

    stateMachine.periodic();

    elevatorMechanism.periodic();
    clawMechanism.periodic();

    Logger.recordOutput("scoring/state", stateMachine.getCurrentState());
  }

  /** This method must be called by RobotContainer, as it does not run automatically! */
  public void testPeriodic() {
    elevatorMechanism.testPeriodic();
    clawMechanism.testPeriodic();
  }
}
