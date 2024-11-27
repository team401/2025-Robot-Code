package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorInputs {
        /** The angle of the 19 tooth-gear-encoder */
        Angle encoder19Angle = Rotations.of(0.0);
        /** The angle of the 17 tooth-gear-encoder */
        Angle encoder17Angle = Rotations.of(0.0);

        /** Goal position of the elevator */
        Distance elevatorGoalHeight = Meters.of(0.0);
        /** Actual position of the elevator */
        Distance elevatorHeight = Meters.of(0.0);

        /** Stator current of the elevator motor */
        Current elevatorStatorCurrent = Amps.of(0.0);
        /** Supply current of the elevator motor */
        Current elevatorSupplyCurrent = Amps.of(0.0);
    }

    @AutoLog
    public static class ElevatorOutputs {
        /** The voltage applied to the elevator motor */
        Voltage elevatorAppliedVolts;
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    public default void applyOutputs(ElevatorOutputs outputs) {}

    /** Seed the elevator's position using Chinese Remainder Theorem. */
    public default void seedWithCRT() {}

    /** Set the goal height which the elevator will control to when it is not in override mode */
    public default void setGoalHeight(Distance goalHeight) {}

    /** Set the static voltage that will be applied when the elevator is in override mode. */
    public default void setOverrideVolts(Voltage volts) {}

    /** Set whether the override voltage should be applied or whether the elevator should control to its position */
    public default void setOverrideMode(boolean override) {}

    /** Update PID gains for the elevator */
    public default void setPID(double p, double i, double d) {}

    /** Set profile constraints to be sent to Motion Magic Expo */
    public default void setMaxProfile(LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {}

    /** Set feedforward gains for closed-loop control */
    public default void setFF(double kS, double kV, double kA, double kG) {}

    /** Set whether or not the motors should brake while idle */
    public default void setBrakeMode(boolean brakeMode) {}

    /** Set the stator current limit for both elevator motors */
    public default void setStatorCurrentLimit(Current currentLimit) {}

    /** Set whether or not the motors on the elevator should be disabled. */
    public default void setMotorsDisabled(boolean disabled) {}
}
