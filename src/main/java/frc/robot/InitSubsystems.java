package frc.robot;

import frc.robot.constants.JsonConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfiguration;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
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
            "Non-exhaustive list of mode types supported in InitSubsystems");
    }

    return new ScoringSubsystem(elevatorMechanism, wristMechanism, clawMechanism);
  }

  public static Drive initDriveSubsystem() {
    switch (Constants.currentMode) {
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
}
