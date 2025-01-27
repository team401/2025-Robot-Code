package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;

public class ClimbSubsystem extends SubsystemBase {

    private ClimbIO io;
    private ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();
    private ClimbOutputsAutoLogged outputs = new ClimbOutputsAutoLogged();

    private BooleanSupplier rampClear = () -> true;

    public enum ClimbAction {
        NONE, // do nothing
        START_CLIMB, // start automated climb sequence
        SYSTEM_READY,
        READY_FOR_CLIMB,
        OVERRIDE
    }

    ClimbAction currentAction = ClimbAction.NONE;

    private enum ClimbState implements StateContainer {
        IDLE(ClimbSubsystem.idle()), // do nothing
        WAITING, // waiting for system to be ready for climb
        SEARCHING, // searching for target
        LIFTING, // moving arm to hang
        OVERRIDE;

        @Override
        public PeriodicStateInterface getState() {
            return state;
        }
    }

    public static StateMachineConfiguration<ClimbState, ClimbAction> climbMachineConfiguration;
    public static StateMachine<ClimbState, ClimbAction> climbMachine;

    public ClimbSubsystem(ClimbIO io) {
        this.io = io;

        climbMachineConfiguration = new StateMachineConfiguration<>();

        climbMachineConfiguration
                .configure(ClimbState.IDLE)
                .permit(ClimbAction.START_CLIMB, ClimbState.WAITING)
                .permit(ClimbAction.OVERRIDE, ClimbState.OVERRIDE);

        climbMachineConfiguration
                .configure(ClimbState.WAITING)
                .permit(ClimbAction.SYSTEM_READY, ClimbState.SEARCHING)
                .permit(ClimbAction.NONE, ClimbState.IDLE);

        climbMachineConfiguration
                .configure(ClimbState.SEARCHING)
                .permit(ClimbAction.READY_FOR_CLIMB, ClimbState.LIFTING)
                .permit(ClimbAction.NONE, ClimbState.IDLE);

        climbMachineConfiguration
                .configure(ClimbState.LIFTING)
                .permit(ClimbAction.NONE, ClimbState.IDLE);

        climbMachine = new StateMachine(climbMachineConfiguration, ClimbAction.NONE);

    }

    public void setAction(ClimbAction action) {
        currentAction = action;
    }

    private void idle() {
    }

    private void waiting() {
        io.setGoalAngle(ClimbConstants.restingAngle);
    }

    private void searching() {
        io.setGoalAngle(ClimbConstants.searchingAngle);
    }

    private void lifting() {
        io.setGoalAngle(ClimbConstants.finalHangingAngle);
    }

    private void override() {
        // idk not my top priority tbh
    }

    @Override
    public void periodic() {

        climbMachine.fire(currentAction);

        io.updateInputs(inputs);
        io.applyOutputs(outputs);

        Logger.processInputs("climb/inputs", inputs);
        Logger.processInputs("climb/outputs", outputs);
        Logger.recordOutput("climb/State", climbMachine.getCurrentState());
        Logger.recordOutput("climb/Action", currentAction);
    }
}
