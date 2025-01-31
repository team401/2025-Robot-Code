package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExampleElevatorCommand;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorMechanism elevatorMechanism;

  public ElevatorSubsystem(ElevatorMechanism elevatorMechanism) {
    this.elevatorMechanism = elevatorMechanism;

    setDefaultCommand(new ExampleElevatorCommand(this));
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

  @Override
  public void periodic() {
    elevatorMechanism.periodic();
  }

  /** This method must be called by RobotContainer, as it does not run automatically! */
  public void testPeriodic() {
    elevatorMechanism.testPeriodic();
  }
}
