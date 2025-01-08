package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriveTemplate extends Subsystem {
  public void setGoalSpeeds(ChassisSpeeds goalSpeeds);
  // hi, ik what ur thinking: just use the one in copppercore. no, because coppercore one has
  // fieldcentric
  // stuff built in, but AK does the field centric conversions in DriveCommands, so we use dis one
  // as a more basic setgoalspeeds
}
