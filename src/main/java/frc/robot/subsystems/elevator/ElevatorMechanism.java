package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorMechanism {
    ElevatorIO io;
    ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
    ElevatorOutputsAutoLogged outputs = new ElevatorOutputsAutoLogged();

    MutDistance goalHeight = Meters.mutable(0.0);
    MutDistance clampedGoalHeight = Meters.mutable(0.0);

    Distance minHeight = ElevatorConstants.minElevatorHeight;
    Distance maxHeight = ElevatorConstants.maxElevatorHeight;

    // Has the elevator been seeded with CRT yet?
    // This exists in case we fail to seed with CRT the first try, it will try again each tick until
    // it succeeds.
    private boolean hasBeenSeeded = false;

    public ElevatorMechanism(ElevatorIO io) {
        this.io = io;

        // Seed elevator height using CRT on initialize
        seedWithCRT();
    }

    public void periodic() {
        if (!hasBeenSeeded) {
            seedWithCRT();
        }

        sendGoalHeightToIO();

        io.updateInputs(inputs);
        io.applyOutputs(outputs);

        Logger.processInputs("elevator/inputs", inputs);
        Logger.processInputs("elevator/outputs", outputs);
    }

    public void seedWithCRT() {
        Logger.recordOutput("elevator/CRTSolutionSpoolAngle", Rotations.of(-1.0));

        final int ticks = ElevatorConstants.CRTticksPerRotation;
        final int smallTeeth = ElevatorConstants.smallCANCoderTeeth;
        final int largeTeeth = ElevatorConstants.largeCANCoderTeeth;
        final int spoolTeeth = ElevatorConstants.spoolTeeth;
        // Find the number of ticks of each encoder, but in terms of the spool.
        // These should be multiplied by 19/18 or 17/18 (the gear ratios of the CANCoders to the
        // spool), but since the resulting numbers
        // aren't divisible by 18, this would result in rounding losing precision.
        // Therefore, we just multiply by 19 or 17 and then divide the final result by
        // 18.
        long ticksSmall =
                Math.round(io.getSmallCANCoderAbsPos().in(Rotations) * ticks * smallTeeth);
        long ticksLarge =
                Math.round(io.getLargeCANCoderAbsPos().in(Rotations) * ticks * largeTeeth);

        long solutionTicks = -1;

        for (int i = 0; i < ticksSmall; i++) {
            // Try the offset of each multiple of 19 * ticks
            long potentialPosition = i * largeTeeth * ticks + ticksLarge;
            // Check whether that potential position is encoder 17's remainder away from a
            // multiple of 17
            if ((potentialPosition - ticksSmall) % (smallTeeth * ticks) == 0) {
                // If both conditions are met, we have a solution.
                solutionTicks = potentialPosition;
                break;
            }
        }

        if (solutionTicks != -1) {
            // Factor out the 18 from earlier.
            Angle solutionSpoolAngle =
                    Rotations.of((double) solutionTicks / (double) ticks / spoolTeeth);
            // The 19 tooth encoder will have turned 18/19 of a rotation for each rotation
            // of the spool
            Angle solutionLargeEncAngle =
                    solutionSpoolAngle.times((double) spoolTeeth / (double) largeTeeth);
            // The 17 tooth encoder will have turned 18/17 of a rotation for each rotation
            // of the spool
            Angle solutionSmallEncAngle =
                    solutionSpoolAngle.times((double) spoolTeeth / (double) smallTeeth);

            // Seed the encoder positions so that they are now accurate
            io.setLargeCANCoderPosition(solutionLargeEncAngle);
            io.setSmallCANCoderPosition(solutionSmallEncAngle);

            hasBeenSeeded = true;

            Logger.recordOutput("elevator/CRTSolutionSpoolAngle", solutionSpoolAngle);
        } else {
            System.out.println("ERROR: Couldn't find solution to seed elevator with CRT");
        }
    }

    /**
     * Set the allowed range of motion for the elevator.
     *
     * <p>When not in override mode, the elevator will clamp its goal height to be within these
     * bounds. If the elevator is outside of these bounds, it will update its goal position and
     * control to be back within these bounds as soon as it can. This can be used to restrict the
     * allowed positions of the elevator, for instance to stop it from destroying a mechanism
     * attached to the elevator when that mechanism is in a certain position.
     */
    public void setAllowedRangeOfMotion(Distance minHeight, Distance maxHeight) {
        this.minHeight = minHeight;
        this.maxHeight = maxHeight;
    }

    public void setMinAllowedHeight(Distance minHeight) {
        this.minHeight = minHeight;
    }

    public void setMaxAllowedHeight(Distance maxHeight) {
        this.maxHeight = maxHeight;
    }

    /**
     * Set the goal height which the elevator will control to when it is not in override mode
     *
     * <p>This goal height will be clamped by the allowed range of motion set by
     * setAllowedRangeOfMotion before it is sent to the elevator io.
     */
    public void setGoalHeight(Distance goalHeight) {
        // TODO: 2025 coppercore solution for easy value clamping
        // https://github.com/team401/coppercore/issues/64
        this.goalHeight.mut_replace(goalHeight);

        sendGoalHeightToIO();
    }

    /**
     * Based on the previously set goal height, update the clamped goal height to be within the
     * current bounds.
     */
    public void updateClampedGoalHeight() {
        if (goalHeight.lt(minHeight)) {
            clampedGoalHeight.mut_replace(minHeight);
        } else if (goalHeight.gt(maxHeight)) {
            clampedGoalHeight.mut_replace(maxHeight);
        } else if (!clampedGoalHeight.equals(goalHeight)) {
            clampedGoalHeight.mut_replace(goalHeight);
        }
    }

    /**
     * Clamp the goal height, then convert it to rotations of the large encoder and send the goal
     * rotations to the IO.
     */
    public void sendGoalHeightToIO() {
        updateClampedGoalHeight();

        // TODO: Use coppercore gear math after https://github.com/team401/coppercore/issues/52 is
        // done.

        Angle spoolRotations = Rotations.of(clampedGoalHeight.divide(Inches.of(4.724)).magnitude());
        Angle largeEncoderRotations =
                spoolRotations.divide(
                        (double) ElevatorConstants.spoolTeeth
                                / (double) ElevatorConstants.largeCANCoderTeeth);

        io.setLargeCANCoderGoalPos(largeEncoderRotations);
    }

    /**
     * Set whether the override voltage should be applied or whether the elevator should control to
     * its position
     */
    public void setOverrideMode(boolean override) {
        io.setOverrideMode(override);
    }

    /** Set the static voltage that will be applied when the elevator is in override mode. */
    public void setOverrideVolts(Voltage volts) {
        io.setOverrideVolts(volts);
    }

    /**
     * Get a reference to the elevator's IO. This should be used to update PID, motion profile, and
     * feed forward gains, and to set brake mode/disable motors. This method exists to avoid the
     * need to duplicate all of these functions between the mechanism and the IO.
     *
     * @return the elevator mechanism's IO
     */
    public ElevatorIO getIO() {
        return io;
    }

    /** Set whether or not the motors on the elevator should be disabled. */
    public void setMotorsDisabled(boolean disabled) {
        io.setMotorsDisabled(disabled);
    }
}
