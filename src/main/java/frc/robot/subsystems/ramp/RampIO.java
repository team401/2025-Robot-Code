package frc.robot.subsystems.ramp;

import org.littletonrobotics.junction.AutoLog;

public interface RampIO {

    @AutoLog
    public static class RampInputs {
        public double position = 0.0;
        public boolean inPosition;
    }

    @AutoLog
    public static class RampOutputs {
        public double targetPosition;
        public double appliedVolts;
    }

    public void updateInputs(RampInputs inputs);

    public void updateOutputs(RampInputs inputs, RampOutputs outputs);
}
