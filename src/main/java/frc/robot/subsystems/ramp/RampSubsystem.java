package frc.robot.subsystems.ramp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO make use PID
// TODO make positions constants
// TODO apply current to hold in position
public class RampSubsystem extends SubsystemBase {

    private RampMechanism mechanism;

    public RampSubsystem(RampMechanism rampMechanism) {
        mechanism = rampMechanism;
    }

    @Override
    public void periodic() {
        mechanism.periodic();
    }

    public void prepareForClimb() {
        mechanism.setPosition(0.0);
    }

    public void prepareForIntake() {
        mechanism.setPosition(3.0);
    }

    public boolean isReadyForClimb() {
        return isReadyForClimb();
    }

    public boolean isReadyForIntake() {
        return isReadyForIntake();
    }
}
