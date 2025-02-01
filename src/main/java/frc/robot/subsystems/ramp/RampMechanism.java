package frc.robot.subsystems.ramp;

import org.littletonrobotics.junction.Logger;

public class RampMechanism {
    RampIO io;
    RampInputsAutoLogged inputs = new RampInputsAutoLogged();
    RampOutputsAutoLogged outputs = new RampOutputsAutoLogged();

    public double position = 1.0;
    public boolean inPosition = false;
    public boolean holdDirectionPositive = false;
    public double positionRange = 0.02;

    public RampMechanism(RampIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        if (position - positionRange <= inputs.position
                && position + positionRange >= inputs.position) {
            inPosition = true;
        } else {
            inPosition = false;
        }
        outputs.targetPosition = position;
        inputs.inPosition = inPosition;
        io.updateOutputs(inputs, outputs);
        Logger.processInputs("ramp/inputs", inputs);
        Logger.processInputs("ramp/outputs", outputs);
    }

    public void setHoldDirection(boolean dir) {}

    public void setPosition(double position) {
        this.position = position;
        this.inPosition = false;
    }

    public boolean inPosition() {
        return inPosition;
    }

    public boolean inTransition() {
        return !inPosition;
    }
}
