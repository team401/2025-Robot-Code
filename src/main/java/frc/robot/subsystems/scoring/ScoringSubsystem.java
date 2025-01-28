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
import frc.robot.subsystems.scoring.states.IntakeCoralState;
import org.littletonrobotics.junction.Logger;

public class ScoringSubsystem extends SubsystemBase {
  private ElevatorMechanism elevatorMechanism;
  private ClawMechanism clawMechanism;

  // Keep track of an instance to pass to state machine
  static ScoringSubsystem instance;

  private enum ScoringStateContainer implements StateContainer {
    Idle(new IdleState(instance)),
    IntakeCoral(new IntakeCoralState(instance));

    private final PeriodicStateInterface state;

    ScoringStateContainer(PeriodicStateInterface state) {
      this.state = state;
    }

    @Override
    public PeriodicStateInterface getState() {
      return state;
    }
  }

  public enum ScoringStateMachineTriggers {
    BeginIntake,
    DoneIntaking,
  }

  private StateMachineConfiguration<ScoringStateContainer, ScoringStateMachineTriggers>
      stateMachineConfiguration;

  private StateMachine<ScoringStateContainer, ScoringStateMachineTriggers> stateMachine;

  public ScoringSubsystem(ElevatorMechanism elevatorMechanism, ClawMechanism clawMechanism) {
    this.elevatorMechanism = elevatorMechanism;
    this.clawMechanism = clawMechanism;

    setDefaultCommand(new ExampleElevatorCommand(this));

    instance = this;

    stateMachineConfiguration = new StateMachineConfiguration<>();

    stateMachineConfiguration
        .configure(ScoringStateContainer.Idle)
        .permit(ScoringStateMachineTriggers.BeginIntake, ScoringStateContainer.IntakeCoral);

    stateMachineConfiguration
        .configure(ScoringStateContainer.IntakeCoral)
        // .permitIf(ScoringStateMachineTriggers.BeginIntake, ScoringStateContainer.Idle, () ->
        // isCoralDetected())
        .permit(ScoringStateMachineTriggers.DoneIntaking, ScoringStateContainer.Idle);

    stateMachine = new StateMachine<>(stateMachineConfiguration, ScoringStateContainer.Idle);

    SmartDashboard.putBoolean("scoring/fireStartIntaking", false);
  }

  /**
   * Fire a trigger on the Scoring state machine
   *
   * @param trigger The trigger to fire.
   */
  public void fireTrigger(ScoringStateMachineTriggers trigger) {
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

  @Override
  public void periodic() {
    boolean fireStartIntaking = SmartDashboard.getBoolean("scoring/fireStartIntaking", false);
    if (fireStartIntaking) {
      stateMachine.fire(ScoringStateMachineTriggers.BeginIntake);
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
