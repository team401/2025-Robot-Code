package frc.robot.subsystems.drive.states;

import java.io.IOException;
import java.util.function.BiConsumer;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.sim.ChassisReference;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FileVersionException;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public class PathFollowState implements PeriodicStateInterface {
  private Drive drive;

  public PathFollowState(Drive drive) {
    this.drive = drive;
  }

  public void periodic() {

    /*FollowPathCommand(
        PathPlannerPath.fromPathFile("Test"), 
        () -> drive.getPose(), 
        () -> drive.getChassisSpeeds(), 
        (ChassisSpeeds speeds, DriveFeedforwards feedforwards)->{
            drive.setGoalSpeeds(speeds, false);
        },
        new PPHolonomicDriveController(
            new PIDConstants(1.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)), Drive.PP_CONFIG, 
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, drive);*/
      
    try {
      AutoBuilder.followPath(PathPlannerPath.fromPathFile("R0 Lineup"));
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }
  }
}
