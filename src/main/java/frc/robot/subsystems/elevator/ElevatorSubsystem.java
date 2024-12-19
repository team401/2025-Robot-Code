package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorMechanism elevatorMechanism;

    public ElevatorSubsystem(ElevatorMechanism elevatorMechanism) {
        this.elevatorMechanism = elevatorMechanism;
        // elevatorMechanism.setGoalHeight(Meters.of(0.75));
    }

    @Override
    public void periodic() {
        elevatorMechanism.periodic();
    }

    /** This method must be called by RobotContainer, as it does not run automatically! */
    public void testPeriodic() {
        elevatorMechanism.testPeriodic();
    }
}
