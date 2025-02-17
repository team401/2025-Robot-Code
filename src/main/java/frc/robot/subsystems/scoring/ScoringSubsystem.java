package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ElevatorIO.ElevatorOutputMode;
import frc.robot.subsystems.scoring.states.IdleState;
import frc.robot.subsystems.scoring.states.InitState;
import frc.robot.subsystems.scoring.states.IntakeState;
import frc.robot.subsystems.scoring.states.ScoreState;
import frc.robot.subsystems.scoring.states.WarmupState;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The scoring subsystem contains the elevator and the wrist, which need to be able to avoid each
 * other and the ground intake.
 *
 * <p>This subsystem functions with a state machine, which uses a series of setpoints defined by
 * ScoringSetpoints.java to configure itself physically for each task required. It uses closed-loop
 * control for the wrist and the elevator, and both the wrist and the elevator implement a system of
 * 'moving clamps' where their position can be 'clamped' regardless of their goal position to ensure
 * that they are out of the way and avoiding collisions when necessary.
 *
 * <p>The general strategy for collision avoidance is as follows:
 *
 * <p>Avoiding collisions with the crossbar:
 *
 * <ul>
 *   <li>If the elevator is below the crossbar and the wrist is in a position where it would
 *       collide, the elevator is clamped to be below the crossbar.
 *   <li>If the elevator is above the crossbar and the wrist is in a position where it would
 *       collide, the elevator is clamped to be above the crossbar.
 *   <li>If the elevator is at the crossbar, the wrist is clamped to be in a position where it
 *       cannot collide.
 *   <li>If the elevator is below the crossbar and its goal position is above the crossbar (or vice
 *       versa), the wrist is clamped to be in a position where it cannot collide.
 *   <li>While these rules do next explicitly force the claw to go to a non-colliding position when
 *       elevator is going up, there will be no setpoint above the crossbar where the wrist is in a
 *       colliding position. Therefore, the wrist will always be moving to a non-colliding position
 *       while the elevator is going up, and the elevator will wait for it to be safe before going
 *       up.
 * </ul>
 *
 * <p>Avoiding collisions with the ground: (this part may or may not be necessary, depending on
 * configuration)
 *
 * <ul>
 *   <li>If the elevator is below a certain height, clamp the wrist angle to be above a certain
 *       value (could be calculated with sin(wristAngle))
 *   <li>If the wrist is below a certain angle, clamp the elevator to be above a certain height to
 *       avoid pushing wrist into ground (could be calculated with sin(wristAngle))
 * </ul>
 *
 * <p>Avoiding collisions with the ground intake:
 *
 * <p>For context, the ground intake should be safely tucked against the elevator except when: 1)
 * elevator is moving past, 2) it is ground intaking
 *
 * <ul>
 *   <li>If the elevator is below the collision point and its goal is to be above the collision
 *       point, clamp the ground intake to be swung out in a non-colliding location
 *   <li>If the elevator is above the collision point and its goal is to be below the collision
 *       point, do the same
 *   <li>When the ground intake is in and the elevator is up, clamp the elevator above it.
 *   <li>When the ground intake is in and the elevator is down, clamp the elevator below it.
 *   <li>These clamps will be automatically reverted as soon as the elevator is past, resulting in,
 *       1) elevator moves as close as it can to the intake 2) intake swings out of the way 3)
 *       elevator moves past 4) intake swings back into place
 * </ul>
 *
 * <p>This multi-clamp system can pose an interesting challenge: how do we choose which clamp to
 * apply? The correct way to solve this is to structure the if-statements in a way that the most
 * restrictive clamps are checked first, so that when those clamps aren't set, the less restrictive
 * clamps are checked. After all of the clamp conditions are checked, we can finally put a call to
 * reset the clamps in an 'else,' allowing a full range of motion when no protection conditions are
 * met.
 */
public class ScoringSubsystem extends SubsystemBase {
  private ElevatorMechanism elevatorMechanism;
  private WristMechanism wristMechanism;
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

  private BooleanSupplier isDriveLinedUpSupplier;

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
    Init(new InitState(instance)),
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
    Seeded,
    BeginIntake,
    DoneIntaking,
    CancelAction,
    ToggleWarmup, // warmup button toggles warmup <-> idle
    StartWarmup, // drive automatically enters warmup when lineup begins
    WarmupReady,
    ScoredPiece,
    ReturnToIdle, // Return to idle when a warmup/score state no longer detects a gamepiece
  }

  private StateMachineConfiguration<ScoringState, ScoringTrigger> stateMachineConfiguration;

  private StateMachine<ScoringState, ScoringTrigger> stateMachine;

  public ScoringSubsystem(
      ElevatorMechanism elevatorMechanism,
      WristMechanism wristMechanism,
      ClawMechanism clawMechanism) {
    this.elevatorMechanism = elevatorMechanism;
    this.wristMechanism = wristMechanism;
    this.clawMechanism = clawMechanism;

    instance = this;

    stateMachineConfiguration = new StateMachineConfiguration<>();

    stateMachineConfiguration
        .configure(ScoringState.Init)
        .permit(ScoringTrigger.Seeded, ScoringState.Idle);

    stateMachineConfiguration
        .configure(ScoringState.Idle)
        .permitIf(
            ScoringTrigger.BeginIntake,
            ScoringState.Intake,
            () -> !(isCoralDetected() || isAlgaeDetected()))
        .permit(ScoringTrigger.ToggleWarmup, ScoringState.Warmup)
        .permitIf(ScoringTrigger.StartWarmup, ScoringState.Warmup, () -> autoTransition);

    stateMachineConfiguration
        .configure(ScoringState.Intake)
        // If autoTransition, go straight to warmup from intake once we're done
        // Otherwise, return to idle
        .permit(ScoringTrigger.DoneIntaking, ScoringState.Idle)
        .permit(ScoringTrigger.CancelAction, ScoringState.Idle);

    stateMachineConfiguration
        .configure(ScoringState.Warmup)
        .permit(ScoringTrigger.ToggleWarmup, ScoringState.Idle)
        // Allow warmup to transition to scoring if autoTransition is enabled and we are lined up
        .permitIf(
            ScoringTrigger.WarmupReady,
            ScoringState.Score,
            () -> autoTransition && isDriveLinedUpSupplier.getAsBoolean())
        .permit(ScoringTrigger.ReturnToIdle, ScoringState.Idle);

    stateMachineConfiguration
        .configure(ScoringState.Score)
        .permit(ScoringTrigger.ScoredPiece, ScoringState.Idle)
        .permit(ScoringTrigger.ReturnToIdle, ScoringState.Idle)
        .permitIf(
            ScoringTrigger.BeginIntake,
            ScoringState.Intake,
            () -> !(clawMechanism.isCoralDetected() || clawMechanism.isAlgaeDetected()));

    stateMachine = new StateMachine<>(stateMachineConfiguration, ScoringState.Init);

    // Manually call the onEntry for init, since we didn't transition into it
    stateMachine.getCurrentState().state.onEntry(null);
  }

  /**
   * Set whether or not the scoring subsystem should automatically transition through its states
   * (e.g. automatically enter warmup when drive starts lining up, automatically go to score when
   * warmup is ready, etc.)
   *
   * @param shouldAutoTransition True if auto transition is enabled, false if not
   */
  public void setAutoTransition(boolean shouldAutoTransition) {
    this.autoTransition = shouldAutoTransition;
  }

  public void setIsDriveLinedUpSupplier(BooleanSupplier newSupplier) {
    isDriveLinedUpSupplier = newSupplier;
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
   * Set whether or not the elevator should override and manually apply a voltage/current, or if it
   * should used closed-loop
   *
   * @param outputMode ClosedLoop, Voltage, or Current
   */
  public void setElevatorOverrideMode(ElevatorOutputMode outputMode) {
    elevatorMechanism.setOutputMode(outputMode);
  }

  public void setElevatorOverrideCurrent(Current current) {
    elevatorMechanism.setOverrideCurrent(current);
  }

  public void setElevatorOverrideVoltage(Voltage volts) {
    elevatorMechanism.setOverrideVoltage(volts);
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
   * Send a new goal angle to the wrist mechanism
   *
   * @param goalAngle Goal wrist to command wrist mechanism to
   */
  public void setWristGoalAngle(Angle goalAngle) {
    if (JsonConstants.scoringFeatureFlags.runWrist) {
      wristMechanism.setGoalAngle(goalAngle);
    }
  }

  /**
   * Set the goal positions for elevator and wrist according to a ScoringSetpoint
   *
   * @param setpoint The setpoint to control to
   */
  public void setGoalSetpoint(ScoringSetpoint setpoint) {
    setElevatorGoalHeight(setpoint.elevatorHeight());
    setWristGoalAngle(setpoint.wristAngle());
    Logger.recordOutput("scoring/setpoint", setpoint.name());
  }

  /**
   * Get the current height of the elevator.
   *
   * <p>This height is determined by the {@link ElevatorMechanism}.
   *
   * @return Current elevator height
   */
  public Distance getElevatorHeight() {
    if (JsonConstants.scoringFeatureFlags.runElevator) {
      return elevatorMechanism.getElevatorHeight();
    } else {
      return Meters.zero();
    }
  }

  /**
   * Get the current velocity of the elevator.
   *
   * <p>This velocity is determined by the {@link ElevatorMechanism}.
   *
   * @return Current elevator velocity
   */
  public LinearVelocity getElevatorVelocity() {
    if (JsonConstants.scoringFeatureFlags.runElevator) {
      return elevatorMechanism.getElevatorVelocity();
    } else {
      return MetersPerSecond.of(0.0);
    }
  }

  /**
   * Get the current angle of the wrist
   *
   * <p>This height is determined by the {@link WristMechanism}.
   *
   * @return
   */
  public Angle getWristAngle() {
    if (JsonConstants.scoringFeatureFlags.runWrist) {
      return wristMechanism.getWristAngle();
    } else {
      return Rotations.zero();
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
      return clawMechanism.isCoralDetected();
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
      return clawMechanism.isAlgaeDetected();
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
    // TODO: find and eliminate cases where null is passed to this function
    if (target != null) {
      currentTarget = target;
    } else {
      System.out.println("WARNING: Null target set in setTarget");
      new Exception("Null target in setTarget").printStackTrace();
    }
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

  /**
   * checks if other subsystems need to wait on intake
   *
   * @return false if intake has coral / algae, and true if its still waiting to intake
   */
  public boolean shouldWaitOnIntake() {
    return !clawMechanism.isCoralDetected() && !clawMechanism.isAlgaeDetected();
  }

  /**
   * checks if other subsystems need to wait on score
   *
   * @return true if scoring subsystem is scoring, and false if it is done
   */
  public boolean shouldWaitOnScore() {
    boolean hasGamePiece;
    switch (currentPiece) {
      case Coral:
      default: // Default only exists so that linter doesn't yell at me for possibly uninitialized
        // hasGamePiece
        hasGamePiece = clawMechanism.isCoralDetected();
        break;
      case Algae:
        hasGamePiece = clawMechanism.isAlgaeDetected();
        break;
    }
    return ((stateMachine.getCurrentState() == ScoringState.Warmup)
            || (stateMachine.getCurrentState() == ScoringState.Score))
        && hasGamePiece;
  }

  @Override
  public void periodic() {
    if (!overrideStateMachine) {
      stateMachine.periodic();
    }

    if (JsonConstants.scoringFeatureFlags.runElevator) {
      elevatorMechanism.periodic();
    }

    if (JsonConstants.scoringFeatureFlags.runWrist) {
      wristMechanism.periodic();
    }

    if (JsonConstants.scoringFeatureFlags.runClaw) {
      clawMechanism.periodic();
    }

    if (JsonConstants.scoringFeatureFlags.runElevator
        && JsonConstants.scoringFeatureFlags.runWrist) {
      determineProtectionClamps();
    }

    Logger.recordOutput("scoring/state", stateMachine.getCurrentState());
  }

  /** This method must be called by RobotContainer, as it does not run automatically! */
  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case ElevatorTuning:
      case WristClosedLoopTuning:
      case WristVoltageTuning:
      case SetpointTuning:
        setOverrideStateMachine(true);
        break;
      default:
        break;
    }

    if (JsonConstants.scoringFeatureFlags.runElevator) {
      elevatorMechanism.testPeriodic();
    }

    if (JsonConstants.scoringFeatureFlags.runWrist) {
      wristMechanism.testPeriodic();
    }

    if (JsonConstants.scoringFeatureFlags.runClaw) {
      clawMechanism.testPeriodic();
    }
  }

  /**
   * Get the Scoring subsystem's ElevatorMechanism instance.
   *
   * <p>This should ONLY be used to create Tunable commands from the elevator.
   *
   * @return the ElevatorMechanism that scoring uses
   */
  public ElevatorMechanism getElevatorMechanismForTuning() {
    return elevatorMechanism;
  }

  /**
   * Has the elevator position been seeded yet?
   *
   * @return True if it has been seeded, false if it hasn't been seeded.
   */
  public boolean hasBeenSeeded() {
    return elevatorMechanism.hasBeenSeeded();
  }

  public void seedElevatorToZero() {
    if (elevatorMechanism != null) {
      elevatorMechanism.seedToZero();
    }
  }

  /**
   * Based on the state of the wrist and elevator, clamp their positions to avoid collisions
   *
   * <p>This method does not verify that the mechanisms exist, so featureflags should be checked
   * before it is called.
   */
  public void determineProtectionClamps() {
    MutDistance elevatorMinHeight = JsonConstants.elevatorConstants.minElevatorHeight.mutableCopy();
    MutDistance elevatorMaxHeight = JsonConstants.elevatorConstants.maxElevatorHeight.mutableCopy();

    Distance elevatorHeight = elevatorMechanism.getElevatorHeight();
    Distance elevatorGoalHeight = elevatorMechanism.getElevatorGoalHeight();

    MutAngle wristMinAngle = JsonConstants.wristConstants.wristMinMinAngle.mutableCopy();
    MutAngle wristMaxAngle = JsonConstants.wristConstants.wristMaxMaxAngle.mutableCopy();

    Angle wristAngle = wristMechanism.getWristAngle();

    // If the elevator is below the minimum safe height for wrist to be down, clamp wrist above its
    // collision point
    if (elevatorHeight.lt(JsonConstants.elevatorConstants.minWristDownHeight)) {
      wristMinAngle.mut_replace(
          (Angle)
              Measure.max(wristMinAngle, JsonConstants.wristConstants.minElevatorDownSafeAngle));
    }

    // If the wrist is below the minimum safe angle for the elevator to be down, clamp the elevator
    // above its collision point
    if (wristAngle.lt(JsonConstants.wristConstants.minElevatorDownSafeAngle)) {
      elevatorMinHeight.mut_replace(
          (Distance)
              Measure.max(elevatorMaxHeight, JsonConstants.elevatorConstants.minWristDownHeight));
    }

    // If the wrist is in an unsafe position for the elevator to move past the crossbar, clamp the
    // elevator above/below its collision point
    if (wristAngle.gt(JsonConstants.wristConstants.maxCrossBarSafeAngle)) {
      if (elevatorHeight.gt(JsonConstants.elevatorConstants.minWristInAboveCrossBarHeight)) {
        elevatorMinHeight.mut_replace(
            (Distance)
                Measure.max(
                    elevatorMinHeight,
                    JsonConstants.elevatorConstants.minWristInAboveCrossBarHeight));
      } else {
        elevatorMaxHeight.mut_replace(
            (Distance)
                Measure.min(
                    elevatorMaxHeight,
                    JsonConstants.elevatorConstants.maxWristInBelowCrossBarHeight));
      }
    }

    // If the elevator is at the height of the crossbar, clamp wrist to be outside collision point
    if (elevatorHeight.gte(JsonConstants.elevatorConstants.maxWristInBelowCrossBarHeight)
        && elevatorHeight.lte(JsonConstants.elevatorConstants.minWristInAboveCrossBarHeight)) {
      wristMaxAngle.mut_replace(
          (Angle) Measure.min(wristMaxAngle, JsonConstants.wristConstants.maxCrossBarSafeAngle));
    }
    // If the elevator is below crossbar and trying to go up or above crossbar and trying to go
    // down, clamp wrist be below its collision point
    if ((elevatorHeight.lte(JsonConstants.elevatorConstants.minWristInAboveCrossBarHeight)
            && elevatorGoalHeight.gte(
                JsonConstants.elevatorConstants.maxWristInBelowCrossBarHeight))
        || (elevatorHeight.gte(JsonConstants.elevatorConstants.maxWristInBelowCrossBarHeight)
            && elevatorGoalHeight.lte(
                JsonConstants.elevatorConstants.maxWristInBelowCrossBarHeight))) {
      wristMaxAngle.mut_replace(
          (Angle) Measure.min(wristMaxAngle, JsonConstants.wristConstants.maxCrossBarSafeAngle));
    }

    elevatorMechanism.setAllowedRangeOfMotion(elevatorMinHeight, elevatorMaxHeight);
    wristMechanism.setAllowedRangeOfMotion(wristMinAngle, wristMaxAngle);
  }

  /**
   * Get the current instance of the scoring subsystem.
   *
   * <p>This should be used for drive to fire the trigger to enter warmup when drive enters lineup
   *
   * @return Current instance of the scoring subsystem.
   */
  public static ScoringSubsystem getInstance() {
    return instance;
  }
}
