package frc.robot;

import frc.robot.constants.ModeConstants;
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
}
