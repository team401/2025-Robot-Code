package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class ScoringSubsystemMapleSim {

  private static Drive drive;
  private static ScoringSubsystem scoring;
  private static SwerveDriveSimulation driveSim;

  private static IntakeSimulation coralIntakeSimulation;
  private static IntakeSimulation algaeIntakeSimulation;

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
                    Meters.of(
                        scoring.getElevatorHeight().in(Meters)
                            + 0.5), // initial height of the ball, in meters
                    MetersPerSecond.of(5), // initial velocity, in m/s
                    scoring.getWristAngle()) // shooter angle
                .withProjectileTrajectoryDisplayCallBack(
                    (poses) ->
                        Logger.recordOutput(
                            "successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
                    (poses) ->
                        Logger.recordOutput(
                            "missedShotsTrajectory", poses.toArray(Pose3d[]::new))));
    // SmartDashboard.putBoolean("clawSim/algaeAvailable", false);
    // SmartDashboard.getBoolean("clawSim/algaeAvailable", false);
  }

  public static void shootCoral() {
    if (drive == null || scoring == null) {
      return;
    }
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new ReefscapeCoralOnFly(
                    drive.getPose().getTranslation(),
                    new Translation2d(0.45, 0),
                    drive.getChassisSpeeds(),
                    drive.getPose().getRotation(),
                    Meters.of(
                        scoring.getElevatorHeight().in(Meters)
                            + 0.5), // initial height of the ball, in meters
                    MetersPerSecond.of(5), // initial velocity, in m/s
                    Degrees.of((scoring.getWristAngle().in(Degrees)) * -1)) // shooter angle
                .withProjectileTrajectoryDisplayCallBack(
                    (poses) -> {},
                    (poses) ->
                        Logger.recordOutput("coralShotsTrajectory", poses.toArray(Pose3d[]::new))));
    // SmartDashboard.putBoolean("clawSim/coralAvailable", false);
    // SmartDashboard.getBoolean("clawSim/coralAvailable", false);
  }

  public static void intakeCoral() {
    if (scoring.isCoralDetected() == false) {
      List<Pose3d> corals = SimulatedArena.getInstance().getGamePiecesByType("Coral");
      int indexOfMinHeightIntake;
      double distanceOfMinHeightIntake = Integer.MAX_VALUE;

      for (int coralIndex = 0; coralIndex < corals.size(); coralIndex++) {
        double heightDistance =
            Math.abs(scoring.getElevatorHeight().in(Meters) + 0.5 - corals.get(coralIndex).getZ());
        if (distanceOfMinHeightIntake > heightDistance) {
          indexOfMinHeightIntake = coralIndex;
          distanceOfMinHeightIntake = heightDistance;
        }
      }

      if (distanceOfMinHeightIntake < 0.1) {
        coralIntakeSimulation.startIntake();
        SmartDashboard.putBoolean("clawSim/coralAvailable", true);
        coralIntakeSimulation.stopIntake();
      }
    }
  }

  public static void intakeAlgae() {
    if (scoring.isCoralDetected() == false) {
      List<Pose3d> algae = SimulatedArena.getInstance().getGamePiecesByType("Algae");
      int indexOfMinHeightIntake;
      double distanceOfMinHeightIntake = Integer.MAX_VALUE;

      for (int algaeIndex = 0; algaeIndex < algae.size(); algaeIndex++) {
        double heightDistance =
            Math.abs(scoring.getElevatorHeight().in(Meters) + 0.5 - algae.get(algaeIndex).getZ());
        if (distanceOfMinHeightIntake > heightDistance) {
          indexOfMinHeightIntake = algaeIndex;
          distanceOfMinHeightIntake = heightDistance;
        }
      }

      if (distanceOfMinHeightIntake < 0.1) {
        algaeIntakeSimulation.startIntake();
        SmartDashboard.putBoolean("clawSim/algaeAvailable", true);
        algaeIntakeSimulation.stopIntake();
      }
    }
  }

  public static void config(
      Drive driveSet, SwerveDriveSimulation driveSimSet, ScoringSubsystem scoringSet) {
    drive = driveSet;
    driveSim = driveSimSet;

    coralIntakeSimulation =
        IntakeSimulation.InTheFrameIntake(
            "Coral", driveSim, Meters.of(0.2), IntakeSimulation.IntakeSide.FRONT, 1);

    algaeIntakeSimulation =
        IntakeSimulation.InTheFrameIntake(
            "Algae", driveSim, Meters.of(0.2), IntakeSimulation.IntakeSide.FRONT, 1);
  }
}
