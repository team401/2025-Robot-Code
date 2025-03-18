package frc.robot.subsystems.drive.states;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StrategyManager;
import frc.robot.constants.AutoStrategyContainer.Action;
import frc.robot.constants.AutoStrategyContainer.ActionType;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import org.littletonrobotics.junction.Logger;

public class OTFState implements PeriodicStateInterface {
  private Drive drive;

  public static Command driveToPose = null;

  public static Pose2d otfPose = null;

  public OTFState(Drive drive) {
    this.drive = drive;
  }

  public void onEntry(Transition transition) {
    if (PathfindingCommand.warmupCommand().isScheduled()) {
      PathfindingCommand.warmupCommand().cancel();
    }

    if (findOTFPoseFromDesiredLocation(drive).equals(otfPose) && driveToPose != null) {
      System.out.println("Scheduling already existing driveToPose command");
      driveToPose.schedule();
      return;
    } else if (driveToPose == null) {
      System.out.println("Not using pre-existing driveToPose command because driveToPose was null");
    } else if (otfPose != findOTFPoseFromDesiredLocation(drive)) {
      System.out.println(
          "Not using pre-existing driveToPose command because otfPose was incorrect");
      System.out.println("  otfPose = " + otfPose.toString());
      System.out.println("  correct otfPose = " + findOTFPoseFromDesiredLocation(drive).toString());
    }

    driveToPose = this.getDriveToPoseCommand(drive);
    System.out.println(driveToPose == null);
    if (driveToPose == null) {
      drive.fireTrigger(DriveTrigger.CancelOTF);
    }

    System.out.println("running on entry correct");
    this.driveToPose.schedule();
  }

  public void onExit(Transition transition) {
    otfPose = null;
    System.out.println("Cancelling command");
    if (driveToPose != null) {
      this.driveToPose.cancel();
    }

    if (DriverStation.isAutonomous() && drive.isGoingToIntake()) {
      drive.setShouldWarmupNextOTF(true);
    }
  }

  public static boolean hasDriveToPoseCommand() {
    return driveToPose != null;
  }

  /**
   * finds a pose to pathfind to based on desiredLocation enum
   *
   * @return a pose representing the corresponding scoring location
   */
  public static Pose2d findOTFPoseFromDesiredLocation(
      DesiredLocation desiredLocation, boolean isAllianceRed) {
    switch (desiredLocation) {
        // NOTE: pairs of reef sides (ie 0 and 1) will have the same otf pose (approximately 0.5-1
        // meter away from center of tag)
      case Reef0:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF0Translation,
                JsonConstants.redFieldLocations.redReefOTF0Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF0Translation,
                JsonConstants.blueFieldLocations.blueReefOTF0Rotation);
      case Reef1:
      case Algae0:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF1Translation,
                JsonConstants.redFieldLocations.redReefOTF1Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF1Translation,
                JsonConstants.blueFieldLocations.blueReefOTF1Rotation);
      case Reef2:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF2Translation,
                JsonConstants.redFieldLocations.redReefOTF2Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF2Translation,
                JsonConstants.blueFieldLocations.blueReefOTF2Rotation);
      case Reef3:
      case Algae1:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF3Translation,
                JsonConstants.redFieldLocations.redReefOTF3Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF3Translation,
                JsonConstants.blueFieldLocations.blueReefOTF3Rotation);
      case Reef4:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF4Translation,
                JsonConstants.redFieldLocations.redReefOTF4Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF4Translation,
                JsonConstants.blueFieldLocations.blueReefOTF4Rotation);
      case Reef5:
      case Algae2:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF5Translation,
                JsonConstants.redFieldLocations.redReefOTF5Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF5Translation,
                JsonConstants.blueFieldLocations.blueReefOTF5Rotation);
      case Reef6:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF6Translation,
                JsonConstants.redFieldLocations.redReefOTF6Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF6Translation,
                JsonConstants.blueFieldLocations.blueReefOTF6Rotation);
      case Reef7:
      case Algae3:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF7Translation,
                JsonConstants.redFieldLocations.redReefOTF7Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF7Translation,
                JsonConstants.blueFieldLocations.blueReefOTF7Rotation);
      case Reef8:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF8Translation,
                JsonConstants.redFieldLocations.redReefOTF8Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF8Translation,
                JsonConstants.blueFieldLocations.blueReefOTF8Rotation);
      case Reef9:
      case Algae4:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF9Translation,
                JsonConstants.redFieldLocations.redReefOTF9Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF9Translation,
                JsonConstants.blueFieldLocations.blueReefOTF9Rotation);
      case Reef10:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF10Translation,
                JsonConstants.redFieldLocations.redReefOTF10Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF10Translation,
                JsonConstants.blueFieldLocations.blueReefOTF10Rotation);
      case Reef11:
      case Algae5:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF11Translation,
                JsonConstants.redFieldLocations.redReefOTF11Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF11Translation,
                JsonConstants.blueFieldLocations.blueReefOTF11Rotation);
      case CoralStationRight:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redCoralStationRightTranslation,
                JsonConstants.redFieldLocations.redCoralStationRightRotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueCoralStationRightTranslation,
                JsonConstants.blueFieldLocations.blueCoralStationRightRotation);
      case CoralStationLeft:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redCoralStationLeftTranslation,
                JsonConstants.redFieldLocations.redCoralStationLeftRotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueCoralStationLeftTranslation,
                JsonConstants.blueFieldLocations.blueCoralStationLeftRotation);
      default:
        return null;
    }
  }

  /**
   * finds a pose to pathfind to based on desiredLocation enum
   *
   * @return a pose representing the corresponding scoring location
   */
  public static Pose2d findOTFPoseFromDesiredLocation(Drive driveInput) {
    return findOTFPoseFromDesiredLocation(
        driveInput.getDesiredLocation(), driveInput.isAllianceRed());
  }

  public static Command getDriveToPoseCommand(
      DesiredLocation location, boolean isGoingToIntake, boolean isAllianceRed) {
    otfPose = findOTFPoseFromDesiredLocation(location, isAllianceRed);

    if (otfPose == null) {
      return null;
    }

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            JsonConstants.drivetrainConstants.OTFMaxLinearVelocity,
            JsonConstants.drivetrainConstants.OTFMaxLinearAccel,
            JsonConstants.drivetrainConstants.OTFMaxAngularVelocity,
            JsonConstants.drivetrainConstants.OTFMaxAngularAccel);

    if (isGoingToIntake) {
      constraints =
          new PathConstraints(
              JsonConstants.drivetrainConstants.OTFMaxLinearVelocity,
              JsonConstants.drivetrainConstants.OTFMaxLinearAccel,
              JsonConstants.drivetrainConstants.OTFMaxAngularVelocity,
              JsonConstants.drivetrainConstants.OTFMaxAngularAccel);
    }

    return AutoBuilder.pathfindToPose(
        otfPose,
        constraints,
        isGoingToIntake ? 0 : JsonConstants.drivetrainConstants.otfPoseEndingVelocity);
  }

  /**
   * gets the path from current pose to the desired pose found from location
   *
   * @return command that drive can schedule to follow the path found
   */
  public static Command getDriveToPoseCommand(Drive drive) {
    otfPose = findOTFPoseFromDesiredLocation(drive);

    if (otfPose == null) {
      return null;
    }

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            JsonConstants.drivetrainConstants.OTFMaxLinearVelocity,
            JsonConstants.drivetrainConstants.OTFMaxLinearAccel,
            JsonConstants.drivetrainConstants.OTFMaxAngularVelocity,
            JsonConstants.drivetrainConstants.OTFMaxAngularAccel);

    if (drive.isGoingToIntake()) {
      constraints =
          new PathConstraints(
              JsonConstants.drivetrainConstants.OTFMaxLinearVelocity,
              JsonConstants.drivetrainConstants.OTFMaxLinearAccel - 1,
              JsonConstants.drivetrainConstants.OTFMaxAngularVelocity,
              JsonConstants.drivetrainConstants.OTFMaxAngularAccel);
    }

    return AutoBuilder.pathfindToPose(
        otfPose,
        constraints,
        drive.isGoingToIntake() ? 0 : JsonConstants.drivetrainConstants.otfPoseEndingVelocity);
  }

  public void periodic() {
    // checks if location has changed (so path can be rescheduled)
    if (otfPose == null || !otfPose.equals(findOTFPoseFromDesiredLocation(drive))) {
      System.out.println("fail on entry");
      this.onEntry(null);
    }

    if (driveToPose != null && !driveToPose.isScheduled()) {
      driveToPose.schedule();
    }

    if (drive.isDriveCloseForWarmup() && ScoringSubsystem.getInstance() != null) {
      ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.StartWarmup);
    } else if (drive.isDriveCloseForFarWarmup() && ScoringSubsystem.getInstance() != null) {
      ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.StartFarWarmup);
    }

    if (driveToPose != null) {
      Logger.recordOutput("Drive/OTF/commandScheduled", driveToPose.isScheduled());
      Logger.recordOutput("Drive/OTF/commandFinished", driveToPose.isFinished());

      if ((driveToPose == null || driveToPose.isFinished())
          && !drive.isDriveCloseToFinalLineupPose()) {
        this.driveToPose = getDriveToPoseCommand(drive);
        if (driveToPose != null) {
          driveToPose.schedule();
        }
      } else if (driveToPose.isFinished() && drive.isDesiredLocationReef()) {
        drive.fireTrigger(DriveTrigger.BeginLineup);
        // go to joystick otherwise
      } else if (driveToPose.isFinished()) {
        drive.fireTrigger(DriveTrigger.CancelOTF);
        System.out.println("driveToPose isFinished canceled OTF!");
      }
    }
  }

  /**
   * Generate the next pathfinding command to generate the command before it is needed
   *
   * <p>This does not schedule the command!
   */
  public static void warmupForNextLocation() {
    Action nextAction = StrategyManager.getInstance().peekNextAction();

    if (nextAction == null || Drive.getInstance() == null) {
      return;
    }

    if (PathfindingCommand.warmupCommand().isScheduled()) {
      PathfindingCommand.warmupCommand().cancel();
    }

    otfPose =
        findOTFPoseFromDesiredLocation(nextAction.location(), Drive.getInstance().isAllianceRed());

    driveToPose =
        getDriveToPoseCommand(
            nextAction.location(),
            nextAction.type() == ActionType.Intake,
            Drive.getInstance().isAllianceRed());

    System.out.println("Warmed up next OTF command with desiredLocation " + nextAction.location());
    System.out.println("  otfPose = " + otfPose.toString());
    // TODO: Figure out if we should call driveToPose.initialize() or driveToPose.execute() to force
    // warmup
  }
}
