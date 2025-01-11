package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.UnitUtils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.TestModeManager;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorMechanism {
    ElevatorIO io;
    ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
    ElevatorOutputsAutoLogged outputs = new ElevatorOutputsAutoLogged();

    MutDistance goalHeight = Meters.mutable(0.0);
    MutDistance clampedGoalHeight = Meters.mutable(0.0);

    Distance minHeight = ElevatorConstants.synced.getObject().minElevatorHeight;
    Distance maxHeight = ElevatorConstants.synced.getObject().maxElevatorHeight;

    LoggedTunableNumber elevatorkP;
    LoggedTunableNumber elevatorkI;
    LoggedTunableNumber elevatorkD;

    LoggedTunableNumber elevatorkS;
    LoggedTunableNumber elevatorkV;
    LoggedTunableNumber elevatorkA;
    LoggedTunableNumber elevatorkG;

    LoggedTunableNumber elevatorExpokV;
    LoggedTunableNumber elevatorExpokA;

    LoggedTunableNumber elevatorTuningSetpointMeters;
    LoggedTunableNumber elevatorTuningOverrideVolts;

    // Has the elevator been seeded with CRT yet?
    // This exists in case we fail to seed with CRT the first try, it will try again each tick until
    // it succeeds.
    private boolean hasBeenSeeded = false;

    public ElevatorMechanism(ElevatorIO io) {
        elevatorkP =
                new LoggedTunableNumber(
                        "ElevatorTunables/elevatorkP",
                        ElevatorConstants.synced.getObject().elevatorkP);
        elevatorkI =
                new LoggedTunableNumber(
                        "ElevatorTunables/elevatorkI",
                        ElevatorConstants.synced.getObject().elevatorkI);
        elevatorkD =
                new LoggedTunableNumber(
                        "ElevatorTunables/elevatorkD",
                        ElevatorConstants.synced.getObject().elevatorkD);

        elevatorkS =
                new LoggedTunableNumber(
                        "ElevatorTunables/elevatorkS",
                        ElevatorConstants.synced.getObject().elevatorkS);
        elevatorkV =
                new LoggedTunableNumber(
                        "ElevatorTunables/elevatorkV",
                        ElevatorConstants.synced.getObject().elevatorkV);
        elevatorkA =
                new LoggedTunableNumber(
                        "ElevatorTunables/elevatorkA",
                        ElevatorConstants.synced.getObject().elevatorkA);
        elevatorkG =
                new LoggedTunableNumber(
                        "ElevatorTunables/elevatorkG",
                        ElevatorConstants.synced.getObject().elevatorkG);

        elevatorExpokV =
                new LoggedTunableNumber(
                        "ElevatorTunables/elevatorExpokV",
                        ElevatorConstants.synced.getObject().elevatorExpo_kV.magnitude());
        elevatorExpokA =
                new LoggedTunableNumber(
                        "ElevatorTunables/elevatorExpokA",
                        ElevatorConstants.synced.getObject().elevatorExpo_kA.magnitude());

        elevatorTuningSetpointMeters =
                new LoggedTunableNumber("ElevatorTunables/elevatorTuningSetpointMeters", 0.0);
        elevatorTuningOverrideVolts =
                new LoggedTunableNumber("ElevatorTunables/elevatorTuningOverrideVolts", 0.0);

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

        Logger.recordOutput("elevator/goalHeight", goalHeight);
        Logger.recordOutput("elevator/clampedGoalHeight", clampedGoalHeight);

        Logger.processInputs("elevator/inputs", inputs);
        Logger.processInputs("elevator/outputs", outputs);
    }

    /** This method must be called from the subsystem's test periodic! */
    public void testPeriodic() {
        switch (TestModeManager.getTestMode()) {
            case ElevatorTuning:
                LoggedTunableNumber.ifChanged(
                        hashCode(),
                        (pid) -> {
                            io.setPID(pid[0], pid[1], pid[2]);
                        },
                        elevatorkP,
                        elevatorkI,
                        elevatorkD);

                LoggedTunableNumber.ifChanged(
                        hashCode(),
                        (ff) -> {
                            io.setFF(ff[0], ff[1], ff[2], ff[3]);
                        },
                        elevatorkS,
                        elevatorkV,
                        elevatorkA,
                        elevatorkG);

                LoggedTunableNumber.ifChanged(
                        hashCode(),
                        (maxProfile) -> {
                            io.setMaxProfile(
                                    RadiansPerSecond.of(0.0),
                                    VoltsPerRadianPerSecondSquared.ofNative(maxProfile[0]),
                                    VoltsPerRadianPerSecond.ofNative(maxProfile[1]));
                        },
                        elevatorExpokA,
                        elevatorExpokV);

                LoggedTunableNumber.ifChanged(
                        hashCode(),
                        (setpoint) -> {
                            setGoalHeight(Meters.of(setpoint[0]));
                        },
                        elevatorTuningSetpointMeters);

                LoggedTunableNumber.ifChanged(
                        hashCode(),
                        (setpoint) -> {
                            io.setOverrideVolts(Volts.of(setpoint[0]));
                        },
                        elevatorTuningOverrideVolts);
                break;
            default:
                break;
        }
    }

    public void seedWithCRT() {
        Logger.recordOutput("elevator/CRTSolutionSpoolAngle", Rotations.of(-1.0));

        final int ticks = ElevatorConstants.synced.getObject().CRTticksPerRotation;
        final int smallTeeth = ElevatorConstants.synced.getObject().smallCANCoderTeeth;
        final int largeTeeth = ElevatorConstants.synced.getObject().largeCANCoderTeeth;
        final int spoolTeeth = ElevatorConstants.synced.getObject().spoolTeeth;
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
            Logger.recordOutput(
                    "elevator/CRTSolutionHeight",
                    Inches.of(solutionSpoolAngle.in(Rotations) * 4.724).in(Meters));
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
        this.goalHeight.mut_replace(goalHeight);

        Logger.recordOutput("elevator/goalHeight", goalHeight);
    }

    /**
     * Based on the previously set goal height, update the clamped goal height to be within the
     * current bounds.
     */
    private void updateClampedGoalHeight() {
        clampedGoalHeight.mut_replace(UnitUtils.clampMeasure(goalHeight, minHeight, maxHeight));
    }

    /**
     * Clamp the goal height, then convert it to rotations of the large encoder and send the goal
     * rotations to the IO.
     */
    private void sendGoalHeightToIO() {
        updateClampedGoalHeight();

        // TODO: Use coppercore gear math after https://github.com/team401/coppercore/issues/52 is
        // done.

        Angle spoolRotations =
                Rotations.of(
                        clampedGoalHeight
                                .divide(
                                        ElevatorConstants.synced.getObject()
                                                .elevatorHeightPerSpoolRotation)
                                .magnitude());
        Angle largeEncoderRotations =
                spoolRotations.times(
                        (double) ElevatorConstants.synced.getObject().spoolTeeth
                                / (double) ElevatorConstants.synced.getObject().largeCANCoderTeeth);

        io.setLargeCANCoderGoalPos(largeEncoderRotations);
    }

    /**
     * Get the current height of the elevator
     *
     * <p>This value is calculated by the current position of the large CANcoder, converted using
     * necessary ratios back into rotations of the spool and then height of the elevator. This means
     * that this value is affected by any error in CRT seed and backlash causing error in large
     * CANcoder position.
     *
     * @return Inferred estimate of the elevator height
     */
    public Distance getElevatorHeight() {
        // Calculate spool rotations by: (largeEncoderPos * largeEncoderTeeth / spoolTeeth)
        Angle spoolAngle =
                inputs.largeEncoderPos.times(
                        (double) ElevatorConstants.synced.getObject().largeCANCoderTeeth
                                / (double) ElevatorConstants.synced.getObject().spoolTeeth);

        // TODO: Use coppercore gear math after https://github.com/team401/coppercore/issues/52 is
        // done.

        // Convert spool rotations to height by multiplying by height per rotation
        return Meters.of(
                spoolAngle.in(Rotations)
                        * ElevatorConstants.synced
                                .getObject()
                                .elevatorHeightPerSpoolRotation
                                .in(Meters));
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
