package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorInputs {
        /** The angle of the 19 tooth-gear-encoder */
        MutAngle encoder19Angle = Rotations.mutable(0.0);
        /** The angle of the 17 tooth-gear-encoder */
        MutAngle encoder17Angle = Rotations.mutable(0.0);

        /** Goal position of the elevator */
        MutDistance elevatorGoalHeight = Meters.mutable(0.0);
        /** Actual position of the elevator */
        MutDistance elevatorHeight = Meters.mutable(0.0);

        /** Stator current of the elevator motor */
        MutCurrent elevatorStatorCurrent = Amps.mutable(0.0);
        /** Supply current of the elevator motor */
        MutCurrent elevatorSupplyCurrent = Amps.mutable(0.0);
    }

    @AutoLog
    public static class ElevatorOutputs {
        /** The voltage applied to the elevator motor */
        MutVoltage elevatorAppliedVolts = Volts.mutable(0.0);
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
