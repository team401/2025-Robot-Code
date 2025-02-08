package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveTrigger;

public class IdleState implements PeriodicStateInterface {
  private Drive drive;

  public IdleState(Drive drive) {
    this.drive = drive;
  }

  public void periodic() {
    drive.setGoalSpeeds(new ChassisSpeeds(), false);

    // set to joysticks once score subsystem is finished
    if (!drive.isWaitingOnScore()) {
      drive.fireTrigger(DriveTrigger.ManualJoysticks);
    }
  }
}
