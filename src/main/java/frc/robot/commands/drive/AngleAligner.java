package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AngleAligner extends Command {
  private Drive drive;
  private double targetAngle;

  public AngleAligner(Drive drive, double reefAngle) {
    this.drive = drive;
    this.targetAngle = reefAngle;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Pass the target angle to the drive subsystem
    drive.setTargetRotation(targetAngle);
  }

  @Override
  public void execute() {
    // Let the drive subsystem handle the rotation control
    drive.alignToTargetRotation();
  }

  @Override
  public boolean isFinished() {
    // Check if the drive subsystem reports the alignment is complete
    return drive.isAlignedToTargetRotation();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drive when the command ends
    drive.stop();
  }
}
