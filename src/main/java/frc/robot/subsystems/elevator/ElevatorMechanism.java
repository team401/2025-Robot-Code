package frc.robot.subsystems.elevator;


import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;

public class ElevatorMechanism {
   ElevatorIO io;
   ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
   ElevatorOutputsAutoLogged outputs = new ElevatorOutputsAutoLogged();

   Distance minHeight = ElevatorConstants.minElevatorHeight;
   Distance maxHeight = ElevatorConstants.maxElevatorHeight;

   public ElevatorMechanism(ElevatorIO io) {
       this.io = io;

       // Seed elevator height using CRT on initialize
       io.seedWithCRT();
   }

   public void periodic() {
       io.updateInputs(inputs);
       io.applyOutputs(outputs);
   }
   
   /**
    * Set the allowed range of motion for the elevator.
    * 
    * <p>
    * When not in override mode, the elevator will clamp its goal height
    * to be within these bounds. If the elevator is outside of these bounds,
    * it will update its goal position and control to be back within these bounds
    * as soon as it can.
    * This can be used to restrict the allowed positions of the elevator,
    * for instance to stop it from destroying a mechanism attached to the elevator
    * when that mechanism is in a certain position.
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
     * <p> This goal height will be clamped by the allowed range of motion set by setAllowedRangeOfMotion before
     * it is sent to the elevator io.
     */
    public void setGoalHeight(Distance goalHeight) {
        // TODO: 2025 coppercore solution for easy value clamping
        if (goalHeight.lt(minHeight)) {
            io.setGoalHeight(minHeight);
        } else if (goalHeight.gt(maxHeight)) {
            io.setGoalHeight(maxHeight);
        } else {
            io.setGoalHeight(goalHeight);
        }
    }

    /** Set whether the override voltage should be applied or whether the elevator should control to its position */
    public void setOverrideMode(boolean override) {
        io.setOverrideMode(override);
    }

    /** Set the static voltage that will be applied when the elevator is in override mode. */
    public void setOverrideVolts(Voltage volts) {
        io.setOverrideVolts(volts);
    }

    /**
     * Get a reference to the elevator's IO.
     * This should be used to update PID, motion profile, and feed forward gains, and to set brake mode/disable motors.
     * This method exists to avoid the need to duplicate all of these functions between the mechanism and the IO.
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
