package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorMechanism elevatorMechanism;

    public ElevatorSubsystem(ElevatorMechanism elevatorMechanism) {
        this.elevatorMechanism = elevatorMechanism;
        elevatorMechanism.setGoalHeight(Meters.of(0.0));
    }

    @Override
    public void periodic() {
        elevatorMechanism.periodic();
    }
}
