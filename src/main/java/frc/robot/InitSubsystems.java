package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import coppercore.vision.VisionIO;
import coppercore.vision.VisionIOPhotonReal;
import coppercore.vision.VisionIOPhotonSim;
import coppercore.vision.VisionLocalizer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfiguration;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOMapleSim;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMapleSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.ramp.RampIOSim;
import frc.robot.subsystems.ramp.RampIOTalonFX;
import frc.robot.subsystems.ramp.RampMechanism;
import frc.robot.subsystems.ramp.RampSubsystem;
import frc.robot.subsystems.scoring.ClawIOSim;
import frc.robot.subsystems.scoring.ClawIOTalonFX;
import frc.robot.subsystems.scoring.ClawMechanism;
import frc.robot.subsystems.scoring.ElevatorIOSim;
import frc.robot.subsystems.scoring.ElevatorIOTalonFX;
import frc.robot.subsystems.scoring.ElevatorMechanism;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.WristIOSim;
import frc.robot.subsystems.scoring.WristIOTalonFX;
import frc.robot.subsystems.scoring.WristMechanism;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public final class InitSubsystems {
  public static ScoringSubsystem initScoringSubsystem() {
    ElevatorMechanism elevatorMechanism = null;
    WristMechanism wristMechanism = null;
    ClawMechanism clawMechanism = null;

    switch (ModeConstants.currentMode) {
      case REAL:
        if (JsonConstants.scoringFeatureFlags.runElevator) {
          elevatorMechanism = new ElevatorMechanism(new ElevatorIOTalonFX());
        }
        if (JsonConstants.scoringFeatureFlags.runWrist) {
          wristMechanism = new WristMechanism(new WristIOTalonFX());
        }
        if (JsonConstants.scoringFeatureFlags.runClaw) {
          clawMechanism = new ClawMechanism(new ClawIOTalonFX());
        }
        break;
      case SIM:
      case MAPLESIM: // TODO: Once ground intake is added, make sure this plays nice with it in
        // maplesim
        if (JsonConstants.scoringFeatureFlags.runElevator) {
          elevatorMechanism = new ElevatorMechanism(new ElevatorIOSim());
        }
        if (JsonConstants.scoringFeatureFlags.runWrist) {
          wristMechanism = new WristMechanism(new WristIOSim());
        }
        if (JsonConstants.scoringFeatureFlags.runClaw) {
          clawMechanism = new ClawMechanism(new ClawIOSim());
        }
        break;
      case REPLAY:
        throw new UnsupportedOperationException("Scoring replay is not yet implemented.");
      default:
        throw new UnsupportedOperationException(
            "Non-exhaustive list of mode types supported in InitSubsystems (got "
                + ModeConstants.currentMode
                + ")");
    }

    return new ScoringSubsystem(elevatorMechanism, wristMechanism, clawMechanism);
  }

  public static ClimbSubsystem initClimbSubsystem() {
    switch (ModeConstants.currentMode) {
      case REAL:
        return new ClimbSubsystem(new ClimbIOTalonFX());
      case SIM:
      case MAPLESIM:
        return new ClimbSubsystem(new ClimbIOSim());
      case REPLAY:
        throw new UnsupportedOperationException("Climb replay is not yet implemented.");
      default:
        throw new UnsupportedOperationException(
            "Non-exhaustive list of mode types supported in InitSubsystems");
    }
  }

  public static Drive initDriveSubsystem() {
    switch (ModeConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(DriveConfiguration.getInstance().FrontLeft),
            new ModuleIOTalonFX(DriveConfiguration.getInstance().FrontRight),
            new ModuleIOTalonFX(DriveConfiguration.getInstance().BackLeft),
            new ModuleIOTalonFX(DriveConfiguration.getInstance().BackRight));

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        return new Drive(
            new GyroIO() {},
            new ModuleIOSim(DriveConfiguration.getInstance().FrontLeft),
            new ModuleIOSim(DriveConfiguration.getInstance().FrontRight),
            new ModuleIOSim(DriveConfiguration.getInstance().BackLeft),
            new ModuleIOSim(DriveConfiguration.getInstance().BackRight));

      case MAPLESIM:

        // Sim robot, instantiate physics sim IO implementations
        RobotContainer.driveSim =
            new SwerveDriveSimulation(
                DriveTrainSimulationConfig.Default().withGyro(COTS.ofPigeon2()),
                new Pose2d(Meters.of(14.350), Meters.of(4.0), new Rotation2d(Degrees.of(180))));
        // DrivetrainConstants.SimConstants.driveSimConfig, new Pose2d());
        SimulatedArena.getInstance().addDriveTrainSimulation(RobotContainer.driveSim);
        return new Drive(
            new GyroIOMapleSim(RobotContainer.driveSim.getGyroSimulation()),
            new ModuleIOMapleSim(
                RobotContainer.driveSim.getModules()[0],
                DriveConfiguration.getInstance().FrontLeft),
            new ModuleIOMapleSim(
                RobotContainer.driveSim.getModules()[1],
                DriveConfiguration.getInstance().FrontRight),
            new ModuleIOMapleSim(
                RobotContainer.driveSim.getModules()[2], DriveConfiguration.getInstance().BackLeft),
            new ModuleIOMapleSim(
                RobotContainer.driveSim.getModules()[3],
                DriveConfiguration.getInstance().BackRight));

      default:
        // Replayed robot, disable IO implementations
        return new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    }
  }

  public static RampSubsystem initRampSubsystem() {
    switch (ModeConstants.currentMode) {
      case REAL:
        return new RampSubsystem(new RampMechanism(new RampIOTalonFX()));
      case SIM:
      case MAPLESIM:
        return new RampSubsystem(new RampMechanism(new RampIOSim()));
      default:
        throw new UnsupportedOperationException(
            "Non-exhaustive list of mode types supported in InitSubsystems");
    }
  }

  public static VisionLocalizer initVisionSubsystem(Drive drive) {
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    switch (ModeConstants.currentMode) {
      case REAL:
        return new VisionLocalizer(
            drive::addVisionMeasurement,
            tagLayout,
            new double[0],
            new VisionIOPhotonReal(
                "Front-Right", JsonConstants.visionConstants.FrontRightTransform));
      case SIM:
        return new VisionLocalizer(
            drive::addVisionMeasurement,
            tagLayout,
            new double[0],
            new VisionIOPhotonSim(
                "Front-Right",
                JsonConstants.visionConstants.FrontRightTransform,
                drive::getPose,
                tagLayout));
      case MAPLESIM:
        return new VisionLocalizer(
            drive::addVisionMeasurement,
            tagLayout,
            new double[0],
            new VisionIOPhotonSim(
                "Front-Right",
                JsonConstants.visionConstants.FrontRightTransform,
                RobotContainer.driveSim::getSimulatedDriveTrainPose,
                tagLayout));
      default:
        return new VisionLocalizer(
            drive::addVisionMeasurement,
            tagLayout,
            new double[0],
            new VisionIO() {},
            new VisionIO() {});
    }
  }

  public static LEDSubsystem initLEDSubsystem() {
    return new LEDSubsystem();
  }
}
