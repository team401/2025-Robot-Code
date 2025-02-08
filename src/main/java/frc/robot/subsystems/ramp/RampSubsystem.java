package frc.robot.subsystems.ramp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.JsonConstants;

// TODO make use PID
// TODO make positions constants
// TODO apply current to hold in position
public class RampSubsystem extends SubsystemBase {

  private RampMechanism mechanism;

  public RampSubsystem(RampMechanism rampMechanism) {
    mechanism = rampMechanism;
  }

  @Override
  public void periodic() {
    mechanism.periodic();
  }

  public void prepareForClimb() {
    mechanism.setPosition(JsonConstants.rampConstants.climbPosition);
  }

  public void prepareForIntake() {
    mechanism.setPosition(JsonConstants.rampConstants.intakePosition);
  }

  public boolean isReadyForClimb() {
    return isReadyForClimb();
  }

  public boolean isReadyForIntake() {
    return isReadyForIntake();
  }
}
