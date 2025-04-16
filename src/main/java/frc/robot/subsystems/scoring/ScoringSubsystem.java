package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import coppercore.wpilib_interface.MonitorWithAlert;
import coppercore.wpilib_interface.MonitoredSubsystem;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.InitBindings;
import frc.robot.TestModeManager;
import frc.robot.TestModeManager.TestMode;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ElevatorIO.ElevatorOutputMode;
import frc.robot.subsystems.scoring.states.FarWarmupState;
import frc.robot.subsystems.scoring.states.IdleState;
import frc.robot.subsystems.scoring.states.InitState;
import frc.robot.subsystems.scoring.states.IntakeState;
import frc.robot.subsystems.scoring.states.ScoreState;
import frc.robot.subsystems.scoring.states.TuningState;
import frc.robot.subsystems.scoring.states.WarmupState;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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
 * <p>Avoiding collisions with the reef: This logic only applies if the robot is within a certain
 * distance of the reef.
 *
 * <ul>
 *   <li>If the elevator goal is above L4 and the elevator is below L4, clamp wrist to Idle position
 *       so it doesn't hit reef on the way up. (This may be unnecessary with new claw geometry)
 *   <li>If the wrist is out beyond a certain angle, clamp elevator above L4 if it's above L4 or
 *       below L4 if it's below L4 until wrist comes in to Idle, so that it doesn't hit the reef on
 *       its way up or down.
 *   <li>If the elevator is below a certain height, clamp the wrist to be up so that it doesn't hit
 *       the base of the reef
 *   <li>If the wrist is at an angle where it would hit the reef base, clamp the elevator above a
 *       certain height
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
public class ScoringSubsystem extends MonitoredSubsystem {
  private ElevatorMechanism elevatorMechanism;
  private WristMechanism wristMechanism;
  private ClawMechanism clawMechanism;

  // Keep track of an instance to pass to state machine
  private static ScoringSubsystem instance;

  /**
   * Keep track of what state to go into after init
   *
   * <p>If a trigger is fired to enter warmup while in init, it will go into warmup. If a trigger
   * for far warmup is fired, it will go into far warmup. Multiple triggers are fired, the most
   * recent trigger will apply.
   */
  private enum StateAfterInit {
    Idle,
    EarlyWarmup,
    Warmup,
    FarWarmup
  }

  private StateAfterInit warmupStateAfterInit = StateAfterInit.Idle;

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

  private FieldTarget currentAlgaeScoreTarget = FieldTarget.Net;
  private FieldTarget currentAlgaeIntakeTarget = FieldTarget.L2;

  public enum GamePiece {
    Coral,
    Algae
  }

  private GamePiece currentPiece = GamePiece.Coral;

  private enum ScoringState implements StateContainer {
    Init(new InitState(instance)),
    Idle(new IdleState(instance)),
    Intake(new IntakeState(instance)),
    FarWarmup(new FarWarmupState(instance)),
    Warmup(new WarmupState(instance)),
    Score(new ScoreState(instance)),
    Tuning(new TuningState());

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
    CancelIntake,
    DoneIntaking,
    StartWarmup, // drive automatically enters warmup when lineup begins
    StartEarlyWarmup, // drive automatically enters warmup when a certain distance away from OTF
    // target (after FarWarmup) ONLY FOR CORAL
    StartFarWarmup, // Start a "far warmup" when drive is a certain distance from it's OTF target
    WarmupReady,
    CancelWarmup, // Warmup button was released, go back to idle
    ScoredPiece,
    ReturnToIdle, // Return to idle when a warmup/score state no longer detects a gamepiece
    EnterTestMode,
    LeaveTestMode,
  }

  private StateMachineConfiguration<ScoringState, ScoringTrigger> stateMachineConfiguration;

  private StateMachine<ScoringState, ScoringTrigger> stateMachine;

  private final BooleanSupplier isScoringTuningSupplier =
      () ->
          DriverStation.isTest()
              && (TestModeManager.getTestMode() == TestMode.ElevatorTuning
                  || TestModeManager.getTestMode() == TestMode.ElevatorCharacterization
                  || TestModeManager.getTestMode() == TestMode.WristClosedLoopTuning
                  || TestModeManager.getTestMode() == TestMode.WristVoltageTuning
                  || TestModeManager.getTestMode() == TestMode.SetpointTuning);

  private Supplier<Distance> reefDistanceSupplier = () -> Meters.zero();

  /**
   * Supplies a boolean determining whether or not the elevator can safely move up without hitting
   * the ramp
   */
  private BooleanSupplier rampSafeSupplier = () -> true;

  public ScoringSubsystem(
      ElevatorMechanism elevatorMechanism,
      WristMechanism wristMechanism,
      ClawMechanism clawMechanism) {
    this.elevatorMechanism = elevatorMechanism;
    this.wristMechanism = wristMechanism;
    this.clawMechanism = clawMechanism;

    if (instance != null) {
      System.out.println("Warning: Instantiated scoring twice!!!!!");
    }

    instance = this;

    stateMachineConfiguration = new StateMachineConfiguration<>();

    stateMachineConfiguration
        .configure(ScoringState.Init)
        .permitIf(ScoringTrigger.Seeded, ScoringState.Tuning, isScoringTuningSupplier)
        .permitIf(
            ScoringTrigger.Seeded,
            ScoringState.Warmup,
            () ->
                !isScoringTuningSupplier.getAsBoolean()
                    && warmupStateAfterInit == StateAfterInit.Warmup)
        .permitIf(
            ScoringTrigger.Seeded,
            ScoringState.Warmup,
            () ->
                !isScoringTuningSupplier.getAsBoolean()
                    && warmupStateAfterInit == StateAfterInit.EarlyWarmup
                    && canFarWarmup())
        .permitIf(
            ScoringTrigger.Seeded,
            ScoringState.FarWarmup,
            () ->
                !isScoringTuningSupplier.getAsBoolean()
                    && warmupStateAfterInit == StateAfterInit.FarWarmup
                    && canFarWarmup())
        .permitIf(
            ScoringTrigger.Seeded,
            ScoringState.Idle,
            () ->
                !isScoringTuningSupplier.getAsBoolean()
                    && warmupStateAfterInit == StateAfterInit.Idle);

    stateMachineConfiguration
        .configure(ScoringState.Idle)
        .permitIf(
            ScoringTrigger.BeginIntake,
            ScoringState.Intake,
            () -> !(isCoralDetected() || isAlgaeDetected()))
        .permitIf(ScoringTrigger.StartEarlyWarmup, ScoringState.Warmup, () -> canFarWarmup())
        .permitIf(ScoringTrigger.StartFarWarmup, ScoringState.FarWarmup, () -> canFarWarmup())
        .permit(ScoringTrigger.StartWarmup, ScoringState.Warmup)
        .permitIf(ScoringTrigger.EnterTestMode, ScoringState.Tuning, isScoringTuningSupplier);

    stateMachineConfiguration
        .configure(ScoringState.Tuning)
        .permit(ScoringTrigger.LeaveTestMode, ScoringState.Idle);

    stateMachineConfiguration
        .configure(ScoringState.Intake)
        .permitIf(
            ScoringTrigger.DoneIntaking,
            ScoringState.Idle,
            () -> currentPiece == GamePiece.Coral || !InitBindings.isIntakeHeld())
        .permit(ScoringTrigger.CancelIntake, ScoringState.Idle);

    stateMachineConfiguration
        .configure(ScoringState.Warmup)
        // Allow warmup to transition to scoring if autoTransition is enabled and we are lined up
        .permitIf(
            ScoringTrigger.WarmupReady,
            ScoringState.Score,
            () -> isDriveLinedUpSupplier.getAsBoolean())
        .permit(ScoringTrigger.ReturnToIdle, ScoringState.Idle)
        .permit(ScoringTrigger.CancelWarmup, ScoringState.Idle);

    stateMachineConfiguration
        .configure(ScoringState.FarWarmup)
        .permit(ScoringTrigger.StartWarmup, ScoringState.Warmup)
        .permit(ScoringTrigger.CancelWarmup, ScoringState.Idle);

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

    if (wristMechanism != null) {
      // Use two monitors: one to alert us if it's temporarily disconnected, one to disable motors
      // if it's disconnected for a long time
      addMonitor(
          new MonitorWithAlert.MonitorWithAlertBuilder()
              .withName("wristEncoderDisconnected")
              .withStickyness(false)
              .withIsStateValidSupplier(() -> wristMechanism.isWristEncoderConnected())
              .withTimeToFault(0.2)
              .withFaultCallback(() -> {})
              .withLoggingEnabled(true)
              .withAlertText("Wrist encoder not connected!")
              .withAlertType(AlertType.kWarning)
              .build());

      // TODO: after https://github.com/team401/coppercore/issues/129, re-enable motors if the
      // encoder comes back.
      addMonitor(
          new MonitorWithAlert.MonitorWithAlertBuilder()
              .withName("wristEncoderDisconnectedExtended")
              .withStickyness(true)
              .withIsStateValidSupplier(
                  () -> (!DriverStation.isEnabled() || wristMechanism.isWristEncoderConnected()))
              .withTimeToFault(2.0)
              .withFaultCallback(
                  () -> {
                    wristMechanism.setMotorsDisabled(true);
                  })
              .withLoggingEnabled(true)
              .withAlertText("Wrist encoder disconnected, motor disabled.")
              .withAlertType(AlertType.kError)
              .build());
    }
  }

  /**
   * Can the scoring subsystem warmup for far warmup at the moment?
   *
   * <p>This is true if the gamepiece is coral and the field target is L3 or L4
   *
   * @return
   */
  public boolean canFarWarmup() {
    return currentPiece == GamePiece.Coral
        && (currentTarget == FieldTarget.L3 || currentTarget == FieldTarget.L4);
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

    if (stateMachine.getCurrentState() == ScoringState.Init) {
      switch (trigger) {
        case StartEarlyWarmup:
          warmupStateAfterInit = StateAfterInit.EarlyWarmup;
        case StartWarmup:
          warmupStateAfterInit = StateAfterInit.Warmup;
          break;
        case StartFarWarmup:
          warmupStateAfterInit = StateAfterInit.FarWarmup;
          break;
        default:
          break;
      }
    }
  }

  public ScoringState getCurrentState() {
    return stateMachine.getCurrentState();
  }

  /** sets brake mode of relevant motors */
  public void setBrakeMode(boolean brake) {
    wristMechanism.setBrakeMode(brake);
    elevatorMechanism.setBrakeMode(brake);
  }

  public void updateScoringLevelFromNetworkTables(
      String coralLevel, String algaeIntakeLevel, String algaeScoreLevel) {
    if (currentPiece == GamePiece.Coral) {
      if (coralLevel.equalsIgnoreCase("-1")) {
        return;
      } else if (coralLevel.equalsIgnoreCase("level1")) {
        this.setTarget(FieldTarget.L1);
      } else if (coralLevel.equalsIgnoreCase("level2")) {
        this.setTarget(FieldTarget.L2);
      } else if (coralLevel.equalsIgnoreCase("level3")) {
        this.setTarget(FieldTarget.L3);
      } else if (coralLevel.equalsIgnoreCase("level4")) {
        this.setTarget(FieldTarget.L4);
      }
    } else if (currentPiece == GamePiece.Algae) {
      if (algaeScoreLevel.equalsIgnoreCase("level1")) {
        currentAlgaeScoreTarget = FieldTarget.Processor;
      } else if (algaeScoreLevel.equalsIgnoreCase("level4")) {
        currentAlgaeScoreTarget = FieldTarget.Net;
      }

      if (algaeIntakeLevel.equalsIgnoreCase("level2")) {
        currentAlgaeIntakeTarget = FieldTarget.L2;
      } else if (algaeIntakeLevel.equalsIgnoreCase("level3")) {
        currentAlgaeIntakeTarget = FieldTarget.L3;
      }
    }
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
   * <p>This angle is determined by the {@link WristMechanism}.
   *
   * @return A Measure, the current Angle of the wrist
   */
  public Angle getWristAngle() {
    if (JsonConstants.scoringFeatureFlags.runWrist) {
      return wristMechanism.getWristAngle();
    } else {
      return Rotations.zero();
    }
  }

  /**
   * Get the current angular velocity of the wrist
   *
   * <p>This velocity is determined by the {@link WristMechanism}
   *
   * @return A Measure, the angular velocity of the wrist
   */
  public AngularVelocity getWristVelocity() {
    if (JsonConstants.scoringFeatureFlags.runWrist) {
      return wristMechanism.getWristVelocity();
    } else {
      return RotationsPerSecond.zero();
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
  public FieldTarget getCoralTarget() {
    return currentTarget;
  }

  public FieldTarget getAlgaeScoreTarget() {
    return currentAlgaeScoreTarget;
  }

  /**
   * Update the algae intake target, should be used in auto since this is done automatically in
   * teleop
   *
   * @param algaeIntakeTarget new algae intake target
   */
  public void setAlgaeIntakeTarget(FieldTarget algaeIntakeTarget) {
    currentAlgaeIntakeTarget = algaeIntakeTarget;
  }

  public FieldTarget getAlgaeIntakeTarget() {
    return currentAlgaeIntakeTarget;
  }

  /**
   * Set which game piece the scoring subsystem should try to intake/score.
   *
   * @param piece The piece type
   */
  public void setGamePiece(GamePiece piece) {
    currentPiece = piece;
    // System.out.println(currentPiece);
    Logger.recordOutput("scoring/gamepiece", piece);
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
    return ((stateMachine.getCurrentState() == ScoringState.Init)
            || (stateMachine.getCurrentState() == ScoringState.Warmup)
            || (stateMachine.getCurrentState() == ScoringState.Score))
        && hasGamePiece;
  }

  @Override
  public void monitoredPeriodic() {
    if (stateMachine.getCurrentState() == ScoringState.Tuning
        && !isScoringTuningSupplier.getAsBoolean()) {
      fireTrigger(ScoringTrigger.LeaveTestMode);
    }
    stateMachine.periodic();

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
    Logger.recordOutput("scoring/isDriveLinedUp", isDriveLinedUpSupplier.getAsBoolean());
  }

  /** This method must be called by RobotContainer, as it does not run automatically! */
  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case SetpointTuning:
      case ElevatorTuning:
      case WristClosedLoopTuning:
      case WristVoltageTuning:
        if (stateMachine.getCurrentState() != ScoringState.Tuning) {
          fireTrigger(ScoringTrigger.EnterTestMode);
        }
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
   * Update the reef distance supplier used for reef collision avoidance
   *
   * @param newSupplier The new Distance supplier, which should supply the robot's distance from
   *     reef center
   */
  public void setReefDistanceSupplier(Supplier<Distance> newSupplier) {
    reefDistanceSupplier = newSupplier;
  }

  /**
   * Update the ramp safety supplier used to keep the elevator below the ramp during climb
   *
   * @param newSupplier The new supplier, which should supply "true" when ramp is in Idle/Intake
   *     position and "false" when ramp is over the top of the elevator for climb.
   */
  public void setRampSafeSupplier(BooleanSupplier newSupplier) {
    rampSafeSupplier = newSupplier;
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
    Angle wristGoalAngle = wristMechanism.getWristGoalAngle();

    boolean wristAboveChassis = false;
    // If the elevator is below the minimum safe height for wrist to be down, clamp wrist above its
    // collision point
    if (elevatorHeight.lt(JsonConstants.elevatorConstants.minWristDownHeight)
        && !isAlgaeDetected()) {
      wristAboveChassis = true;
      wristMinAngle.mut_replace(
          (Angle)
              Measure.max(wristMinAngle, JsonConstants.wristConstants.minElevatorDownSafeAngle));
    }
    Logger.recordOutput("scoring/clamps/wristAboveChassis", wristAboveChassis);

    boolean elevatorAboveClaw = false;
    // If the wrist is below the minimum safe angle for the elevator to be down, clamp the elevator
    // above its collision point
    if (wristAngle.lt(JsonConstants.wristConstants.minElevatorDownSafeAngle)) {
      elevatorAboveClaw = true;
      elevatorMinHeight.mut_replace(
          (Distance)
              Measure.max(elevatorMinHeight, JsonConstants.elevatorConstants.minWristDownHeight));
    }
    Logger.recordOutput("scoring/clamps/elevatorAboveClaw", elevatorAboveClaw);

    boolean closeToReef = false;
    boolean wristInToAvoidReefBase = false;
    boolean elevatorUpToAvoidReefBase = false;
    boolean wristInToPassReef = false;
    boolean elevatorBelowReefLevel = false;
    boolean elevatorAboveReefLevel = false;

    Distance reefDistance = reefDistanceSupplier.get();
    Logger.recordOutput("scoring/reefDistanceSupplier", reefDistance);
    if (reefDistance.lt(JsonConstants.wristConstants.closeToReefThreshold)
        && !DriverStation.isTest()) {
      closeToReef = true;

      if (elevatorHeight.lte(JsonConstants.elevatorConstants.minReefSafeHeight)
          && currentPiece != GamePiece.Algae) {
        wristInToAvoidReefBase = true;
        // If elevator is next to reef base, make sure wrist doesn't hit it
        wristMinAngle.mut_replace(
            (Angle) Measure.max(wristMinAngle, JsonConstants.wristConstants.minReefSafeAngle));
      }

      if (getGamePiece() == GamePiece.Coral
          && ReefAvoidanceHelper.willPassReefLevel(elevatorHeight, elevatorGoalHeight)) {
        // Don't run this protection when holding an algae
        wristInToPassReef = true;
        // If we will pass a reef level, clamp the wrist to be in a safe position to pass the reef
        wristMinAngle.mut_replace(
            (Angle) Measure.max(wristMinAngle, JsonConstants.wristConstants.minReefSafeAngle));

        // If we will pass a reef level and the wrist is in an unsafe position to pass the reef,
        // clamp
        // the elevator above or below the point of collision
        if (wristAngle.lt(JsonConstants.wristConstants.minReefSafeAngle)) {
          elevatorUpToAvoidReefBase = true;
          // If the wrist would hit the reef base, clamp the elevator above the reef base height
          elevatorMinHeight.mut_replace(
              (Distance)
                  Measure.max(
                      elevatorMinHeight, JsonConstants.elevatorConstants.minReefSafeHeight));

          if (elevatorHeight.lt(elevatorGoalHeight)) {
            elevatorBelowReefLevel = true;
            elevatorMaxHeight.mut_replace(
                (Distance)
                    Measure.min(
                        elevatorMaxHeight,
                        ReefAvoidanceHelper.getCollisionHeight(
                            elevatorHeight, elevatorGoalHeight)));
          } else {
            elevatorAboveReefLevel = true;
            elevatorMinHeight.mut_replace(
                (Distance)
                    Measure.max(
                        elevatorMinHeight,
                        ReefAvoidanceHelper.getCollisionHeight(
                            elevatorHeight, elevatorGoalHeight)));
          }
        }
      }
    }

    boolean wristDownForAlgae = false;
    if (isAlgaeDetected()
        && elevatorMechanism
            .getElevatorHeight()
            .lt(JsonConstants.elevatorConstants.minAlgaeInHeight)) {
      wristDownForAlgae = true;
      wristMaxAngle.mut_replace(
          (Angle) Measure.min(wristMaxAngle, JsonConstants.wristConstants.algaeUnderCrossbarAngle));
    }

    boolean wristInDangerZone =
        wristAngle.gt(JsonConstants.wristConstants.crossbarBottomCollisionAngle)
            && wristAngle.lt(JsonConstants.wristConstants.crossbarTopCollisionAngle);
    boolean willPassDangerZoneUp =
        wristAngle.lt(JsonConstants.wristConstants.crossbarBottomCollisionAngle)
            && wristGoalAngle.gt(JsonConstants.wristConstants.crossbarBottomCollisionAngle);
    boolean willPassDangerZoneDown =
        wristAngle.gt(JsonConstants.wristConstants.crossbarTopCollisionAngle)
            && wristGoalAngle.lt(JsonConstants.wristConstants.crossbarTopCollisionAngle);
    boolean willPassDangerZone = willPassDangerZoneUp || willPassDangerZoneDown;

    boolean canWristHitCrossbar = wristInDangerZone || willPassDangerZone;

    if (canWristHitCrossbar) {
      // If the wrist can hit the crossbar, keep elevator on the same side of the crossbar that it
      // is
      if (elevatorHeight.lt(JsonConstants.elevatorConstants.crossbarTopCollisionHeight)) {
        elevatorMaxHeight.mut_replace(
            (Distance)
                Measure.min(
                    elevatorMaxHeight,
                    JsonConstants.elevatorConstants.crossbarBottomCollisionHeight));
      } else {
        elevatorMinHeight.mut_replace(
            (Distance)
                Measure.max(
                    elevatorMinHeight, JsonConstants.elevatorConstants.crossbarTopCollisionHeight));
      }
    }

    Logger.recordOutput("scoring/clamps/closeToReef", closeToReef);
    Logger.recordOutput("scoring/clamps/wristInToAvoidReefBase", wristInToAvoidReefBase);
    Logger.recordOutput("scoring/clamps/elevatorUpToAvoidReefBase", elevatorUpToAvoidReefBase);
    Logger.recordOutput("scoring/clamps/wristInToPassReef", wristInToPassReef);
    Logger.recordOutput("scoring/clamps/elevatorBelowReefLevel", elevatorBelowReefLevel);
    Logger.recordOutput("scoring/clamps/elevatorAboveReefLevel", elevatorAboveReefLevel);
    Logger.recordOutput("scoring/clamps/wristDownForAlgae", wristDownForAlgae);
    Logger.recordOutput("scoring/clamps/canWristHitCrossbar", canWristHitCrossbar);

    elevatorMechanism.setAllowedRangeOfMotion(elevatorMinHeight, elevatorMaxHeight);
    wristMechanism.setAllowedRangeOfMotion(wristMinAngle, wristMaxAngle);
  }

  /**
   * Set the supplier used to get the value of the joystick used to move the elevator setpoint in
   * setpoint tuning mode
   */
  public void setTuningHeightSetpointAdjustmentSupplier(DoubleSupplier newSupplier) {
    if (elevatorMechanism != null) {
      elevatorMechanism.setTuningHeightSetpointAdjustmentSupplier(newSupplier);
    }
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
