package frc.robot.subsystems.ramp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        mechanism.setAction(RampMechanism.Action.CLIMB);
    }

    public void prepareForIntake() {
        mechanism.setAction(RampMechanism.Action.INTAKE);
    }

    public boolean isReadyForClimb() {
        return isReadyForClimb();
    }

    public boolean isReadyForIntake() {
        return isReadyForIntake();
    }
}
