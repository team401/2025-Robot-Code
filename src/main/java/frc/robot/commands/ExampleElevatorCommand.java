package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the elevator to a setpoint, waits for it to achieve it, and then slightly increases the
 * setpoint and waits again. Aims to target roughly 3 setpoints within in each range of motion of
 * the elevator, and then wraps its setpoints around the max position to control back to the bottom.
 */
public class ExampleElevatorCommand extends Command {
    ElevatorSubsystem elevatorSubsystem;

    MutDistance currentGoalHeight = Meters.mutable(0.0);
    // Start with getting max range of elevator (max height - min height), then divide by 3 to
    // control to 3 different setpoints along elevator's height. Then, subtract a small number so it
    // will wrap around to hit more varied setpoints.
    Distance stepHeight =
            ElevatorConstants.synced
                    .getObject()
                    .maxElevatorHeight
                    .minus(ElevatorConstants.synced.getObject().minElevatorHeight)
                    .div(3.0)
                    .minus(Meters.of(0.1));

    /*
     * The threshold for how close the elevator must get to the setpoint before it is considered to be "at" the setpoint
     *
     * <p> This isn't an ElevatorConstant because this command is just an example and only exists to prove that the elevator works.
     */
    final double setpointThresholdMeters = 0.01;

    public ExampleElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (Math.abs(
                        elevatorSubsystem.getElevatorHeight().in(Meters)
                                - currentGoalHeight.in(Meters))
                < setpointThresholdMeters) {
            // We're at the setpoint, so increment it.
            incrementGoalHeight();
        }

        elevatorSubsystem.setElevatorGoalHeight(currentGoalHeight);

        Logger.recordOutput("ExampleElevatorCommand/goalHeight", currentGoalHeight);
    }

    /**
     * Increment the goal height by stepHeight, wrapping it around to the bottom of the elevator's
     * range of motion if it exceeds the maximum height.
     */
    private void incrementGoalHeight() {
        currentGoalHeight.mut_plus(stepHeight);

        // If current goal height is above max height, wrap around to the bottom of the range of
        // motion
        if (currentGoalHeight.gt(ElevatorConstants.synced.getObject().maxElevatorHeight)) {
            Distance rangeOfMotion =
                    ElevatorConstants.synced
                            .getObject()
                            .maxElevatorHeight
                            .minus(ElevatorConstants.synced.getObject().minElevatorHeight);
            Distance goalHeightFromBottom =
                    currentGoalHeight.minus(ElevatorConstants.synced.getObject().minElevatorHeight);
            // There seems to be no way to do a modulus supplied by units library. Therefore, we
            // have to convert out to meters, mod, and then create a new distance.
            goalHeightFromBottom =
                    Meters.of(goalHeightFromBottom.in(Meters) % rangeOfMotion.in(Meters));

            currentGoalHeight.mut_replace(
                    goalHeightFromBottom.plus(
                            ElevatorConstants.synced.getObject().minElevatorHeight));
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        // This command runs forever
        return false;
    }
}
