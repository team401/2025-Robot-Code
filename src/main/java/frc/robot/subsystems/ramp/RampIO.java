package frc.robot.subsystems.ramp;

import org.littletonrobotics.junction.AutoLog;

public interface RampIO {

    @AutoLog
    public static class RampInputs {
        public double position = 0.0;
    }

    @AutoLog
    public static class RampOutputs {
        public double appliedVolts;
    }

    public default void periodic(RampInputsAutoLogged inputs, RampOutputsAutoLogged outputs) {
        updateInputs(inputs);
        updateOutputs(outputs);
    }

    public void updateInputs(RampInputsAutoLogged inputs);

    public void updateOutputs(RampOutputsAutoLogged outputs);
}
