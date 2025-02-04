package frc.robot;

import frc.robot.constants.JsonConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorMechanism;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public final class InitSubsystems {
  public static ElevatorSubsystem initElevatorSubsystem() {
    switch (ModeConstants.currentMode) {
      case REAL:
        return new ElevatorSubsystem(new ElevatorMechanism(new ElevatorIOTalonFX()));
      case SIM:
        return new ElevatorSubsystem(new ElevatorMechanism(new ElevatorIOSim()));
      case REPLAY:
        throw new UnsupportedOperationException("Elevator replay is not yet implemented.");
      default:
        throw new UnsupportedOperationException(
            "Non-exhaustive list of mode types supported in InitSubsystems");
    }
  }

  public static ClimbSubsystem initClimbSubsystem() {
    switch (ModeConstants.currentMode) {
      case REAL:
        throw new UnsupportedOperationException("Climb real functions are not yet implemented.");
      case SIM:
        return new ClimbSubsystem(new ClimbIOSim());
      case REPLAY:
        throw new UnsupportedOperationException("Climb replay is not yet implemented.");
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
