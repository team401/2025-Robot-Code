package frc.robot.subsystems.drive.states;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;

public class OTFState implements PeriodicStateInterface {
  private Drive drive;

  private Command driveToPose = null;

  public OTFState(Drive drive) {
    this.drive = drive;
  }

  public void onEntry(Transition transition) {
    PathfindingCommand.warmupCommand().cancel();
    driveToPose = this.getDriveToPoseCommand();
    this.driveToPose.schedule();
  }

  public void onExit(Transition transition) {
    if (driveToPose != null) {
      this.driveToPose.cancel();
    }
  }

  /**
   * finds a pose to pathfind to based on desiredLocation enum
   *
   * @return a pose representing the corresponding scoring location
   */
  public static Pose2d findOTFPoseFromDesiredLocation(Drive driveInput) {
    switch (driveInput.getDesiredLocation()) {
        // NOTE: pairs of reef sides (ie 0 and 1) will have the same otf pose (approximately 0.5-1
        // meter away from center of tag)
      case Reef0:
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? JsonConstants.redFieldLocations.reef0
            : new Pose2d();
      case Reef1:
        return new Pose2d(Meters.of(14.350), Meters.of(4.0), new Rotation2d(Degrees.of(180)));
      case CoralStationRight:
        return new Pose2d(16.0, 6.6, new Rotation2d(0.0));
        // return new Pose2d(1.2, 1, Rotation2d.fromRadians(1));
      case CoralStationLeft:
        return new Pose2d(1.2, 7.0, Rotation2d.fromRadians(-1));
      default:
        // no location set, exit state?
        return null;
    }
  }

  /**
   * gets the path from current pose to the desired pose found from location
   *
   * @return command that drive can schedule to follow the path found
   */
  public Command getDriveToPoseCommand() {
    Pose2d targetPose = findOTFPoseFromDesiredLocation(drive);

    if (targetPose == null) {
      return null;
    }

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            JsonConstants.drivetrainConstants.OTFMaxLinearVelocity,
            JsonConstants.drivetrainConstants.OTFMaxLinearAccel,
            JsonConstants.drivetrainConstants.OTFMaxAngularVelocity,
            JsonConstants.drivetrainConstants.OTFMaxAngularAccel);

    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }

  public void periodic() {}
}
