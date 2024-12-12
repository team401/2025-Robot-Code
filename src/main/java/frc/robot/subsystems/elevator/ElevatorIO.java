package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorInputs {
        /**
         * The angle of the 19 tooth-gear-encoder. Counts total rotations, not absolute position.
         */
        MutAngle largeEncoderPos = Rotations.mutable(0.0);

        /**
         * The angle of the 17 tooth-gear-encoder. Counts total rotations, not absolute position.
         */
        MutAngle smallEncoderPos = Rotations.mutable(0.0);

        /** Absolute position of the 19 tooth-gear-encocer. Wraps around after 1 rotation */
        MutAngle largeEncoderAbsolutePos = Rotations.mutable(0.0);

        /** Absolute position of the 17 tooth-gear-encocer. Wraps around after 1 rotation */
        MutAngle smallEncoderAbsolutePos = Rotations.mutable(0.0);

        /** Goal position of the elevator */
        MutAngle largeEncoderGoalPos = Rotations.mutable(0.0);

        /** Profile setpoint goal position of the elevator */
        MutAngle largeEncoderSetpointPos = Rotations.mutable(0.0);

        /** Stator current of the lead elevator motor */
        MutCurrent elevatorLeadMotorStatorCurrent = Amps.mutable(0.0);

        /** Supply current of the lead elevator motor */
        MutCurrent elevatorLeadMotorSupplyCurrent = Amps.mutable(0.0);

        /** Stator current of the follower elevator motor */
        MutCurrent elevatorFollowerMotorStatorCurrent = Amps.mutable(0.0);

        /** Supply current of the follower elevator motor */
        MutCurrent elevatorFollowerMotorSupplyCurrent = Amps.mutable(0.0);

        double motionMagicError = 0.0;
    }

    @AutoLog
    public static class ElevatorOutputs {
        /** The voltage applied to the elevator motor */
        MutVoltage elevatorAppliedVolts = Volts.mutable(0.0);
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    public default void applyOutputs(ElevatorOutputs outputs) {}

    /**
     * Set the goal position of CANCoder 19 which the elevator will control to when it is not in
     * override mode
     */
    public default void setLargeCANCoderGoalPos(Angle goalPos) {}

    /** Get the absolute position of the 19 tooth CANCoder. */
    public default Angle getLargeCANCoderAbsPos() {
        return Rotations.of(0.0);
    }

    /** Get the absolute position of the 17 tooth CANCoder. */
    public default Angle getSmallCANCoderAbsPos() {
        return Rotations.of(0.0);
    }

    /**
     * Set the position of the 19 tooth CANCoder. This position is separate from absolute position
     * and can track multiple rotations.
     */
    public default void setLargeCANCoderPosition(Angle newAngle) {}

    /**
     * Set the position of the 17 tooth CANCoder. This position is separate from absolute position
     * and can track multiple rotations.
     */
    public default void setSmallCANCoderPosition(Angle newAngle) {}

    /** Set the static voltage that will be applied when the elevator is in override mode. */
    public default void setOverrideVolts(Voltage volts) {}

    /**
     * Set whether the override voltage should be applied or whether the elevator should control to
     * its position
     */
    public default void setOverrideMode(boolean override) {}

    /** Update PID gains for the elevator */
    public default void setPID(double p, double i, double d) {}

    /** Set profile constraints to be sent to Motion Magic Expo */
    public default void setMaxProfile(
            AngularVelocity maxVelocity,
            Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
            Per<VoltageUnit, AngularVelocityUnit> expo_kV) {}

    /** Set feedforward gains for closed-loop control */
    public default void setFF(double kS, double kV, double kA, double kG) {}

    /** Set whether or not the motors should brake while idle */
    public default void setBrakeMode(boolean brakeMode) {}

    /** Set the stator current limit for both elevator motors */
    public default void setStatorCurrentLimit(Current currentLimit) {}

    /** Set whether or not the motors on the elevator should be disabled. */
    public default void setMotorsDisabled(boolean disabled) {}
}
