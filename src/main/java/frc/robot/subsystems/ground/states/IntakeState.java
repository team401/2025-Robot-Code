package frc.robot.subsystems.ground.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.subsystems.ground.GroundIntakeSubsystem;

public class IntakeState implements PeriodicStateInterface {
  private GroundIntakeSubsystem groundIntakeSubsystem;

  public IntakeState(GroundIntakeSubsystem groundIntakeSubsystem) {
    this.groundIntakeSubsystem = groundIntakeSubsystem;
  }

  @Override
  public void periodic() {}
}
