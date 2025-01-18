package frc.robot;

import frc.robot.constants.ModeConstants;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbSubsystem;
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
                throw new UnsupportedOperationException(
                        "Climb real functions are not yet implemented.");
            case SIM:
                return new ClimbSubsystem(new ClimbIOSim());
            case REPLAY:
                throw new UnsupportedOperationException("Climb replay is not yet implemented.");
            default:
                throw new UnsupportedOperationException(
                        "Non-exhaustive list of mode types supported in InitSubsystems");
        }
    }
}
