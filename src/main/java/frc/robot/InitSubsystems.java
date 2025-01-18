package frc.robot;

import frc.robot.constants.ModeConstants;
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
}
