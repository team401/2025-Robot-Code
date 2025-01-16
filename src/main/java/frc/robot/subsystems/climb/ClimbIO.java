package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog
    public static class ClimbInputs {

        boolean active = false;
        boolean lockedToCage = false;

        MutVoltage motorVoltage = Volts.mutable(0);

        MutAngle motorAngle = Radians.mutable(0);
    }

    @AutoLog
    public static class ClimbOutputs {

        MutVoltage goalVoltage = Volts.mutable(0);

        MutAngle goalAngle = Radians.mutable(0);
    }

    public default void updateInputs(ClimbInputs inputs) {}

    public default void applyOutputs(ClimbOutputs outputs) {}

    public default void applyStatus(boolean active) {}

    public default void setVoltage() {}

    public default void setGoalAngle() {}

    public default void setPID(double p, double i, double d) {}

    public default void setFF(double kS, double kV, double kA, double kG) {}
}
