package frc.robot;

import frc.robot.constants.JsonConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.subsystems.drive.Drive;
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

public final class InitSubsystems {
    public static ScoringSubsystem initScoringSubsystem() {
        switch (ModeConstants.currentMode) {
            case REAL:
                return new ScoringSubsystem(
                        new ElevatorMechanism(new ElevatorIOTalonFX()),
                        new ClawMechanism(new ClawIOTalonFX()));
            case SIM:
                return new ScoringSubsystem(
                        new ElevatorMechanism(new ElevatorIOSim()),
                        new ClawMechanism(new ClawIOSim()));
            case REPLAY:
                throw new UnsupportedOperationException("Elevator replay is not yet implemented.");
            default:
                throw new UnsupportedOperationException(
                        "Non-exhaustive list of mode types supported in InitSubsystems");
        }
    }



  public static Drive initDriveSubsystem() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        return new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(JsonConstants.drivetrainConstants.FrontLeft),
            new ModuleIOTalonFX(JsonConstants.drivetrainConstants.FrontRight),
            new ModuleIOTalonFX(JsonConstants.drivetrainConstants.BackLeft),
            new ModuleIOTalonFX(JsonConstants.drivetrainConstants.BackRight));

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        return new Drive(
            new GyroIO() {},
            new ModuleIOSim(JsonConstants.drivetrainConstants.FrontLeft),
            new ModuleIOSim(JsonConstants.drivetrainConstants.FrontRight),
            new ModuleIOSim(JsonConstants.drivetrainConstants.BackLeft),
            new ModuleIOSim(JsonConstants.drivetrainConstants.BackRight));

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
