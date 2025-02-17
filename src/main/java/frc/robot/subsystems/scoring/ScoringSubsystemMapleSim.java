package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class ScoringSubsystemMapleSim {

  public static Drive drive;
  public static ScoringSubsystem scoring;

  public static void shootAlgae() {
    if (drive == null || scoring == null) {
      return;
    }
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new ReefscapeAlgaeOnFly(
                    drive.getPose().getTranslation(),
                    new Translation2d(0.5, 0),
                    drive.getChassisSpeeds(),
                    drive.getPose().getRotation(),
                    scoring.getElevatorHeight(), // initial height of the ball, in meters
                    MetersPerSecond.of(5), // initial velocity, in m/s
                    scoring.getWristAngle()) // shooter angle
                .withProjectileTrajectoryDisplayCallBack(
                    (poses) ->
                        Logger.recordOutput(
                            "successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
                    (poses) ->
                        Logger.recordOutput(
                            "missedShotsTrajectory", poses.toArray(Pose3d[]::new))));
  }

  public static void shootCoral() {
    if (drive == null || scoring == null) {
      return;
    }
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new ReefscapeCoralOnFly(
                    drive.getPose().getTranslation(),
                    new Translation2d(0.5, 0),
                    drive.getChassisSpeeds(),
                    drive.getPose().getRotation(),
                    scoring.getElevatorHeight(), // initial height of the ball, in meters
                    MetersPerSecond.of(5), // initial velocity, in m/s
                    scoring.getWristAngle()) // shooter angle
                .withProjectileTrajectoryDisplayCallBack(
                    (poses) ->
                        Logger.recordOutput(
                            "successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
                    (poses) ->
                        Logger.recordOutput(
                            "missedShotsTrajectory", poses.toArray(Pose3d[]::new))));
  }

  public static void configDrive(Drive driveSet) {
    drive = driveSet;
  }

  public static void configScoring(ScoringSubsystem scoringSet) {
    scoring = scoringSet;
  }
}
