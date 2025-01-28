package frc.robot.subsystems.climb;

import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

    private ClimbIO io;
    private ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();
    private ClimbOutputsAutoLogged outputs = new ClimbOutputsAutoLogged();

    private BooleanSupplier rampClear = () -> true;

    static ClimbSubsystem instance;

    public enum ClimbAction {
        NONE, // do nothing
        START_CLIMB, // start automated climb sequence
        SYSTEM_READY,
        READY_FOR_CLIMB,
        OVERRIDE
    }

    ClimbAction currentAction = ClimbAction.NONE;

    private static enum ClimbState implements StateContainer {
        IDLE(new IdleState()), // do nothing
        WAITING(new WaitingState()), // waiting for system to be ready for climb
        SEARCHING(new SearchingState()), // searching for target
        LIFTING(new LiftingState()), // moving arm to hang
        OVERRIDE(new PeriodicStateInterface() {});

        private final PeriodicStateInterface state;

        ClimbState(PeriodicStateInterface state) {
            this.state = state;
          }

        @Override
        public PeriodicStateInterface getState() {
            return state;
        }
    }

    public class IdleState implements PeriodicStateInterface {
        @Override
        public void periodic() {
            io.setGoalAngle(ClimbConstants.restingAngle);
        }
    }

    public class WaitingState implements PeriodicStateInterface {
        @Override
        public void periodic() {
            io.setGoalAngle(ClimbConstants.restingAngle);
            if (rampClear.getAsBoolean()) climbMachine.fire(ClimbAction.SYSTEM_READY);
        }
    }

    public class SearchingState implements PeriodicStateInterface {
        @Override
        public void periodic() {
            io.setGoalAngle(ClimbConstants.searchingAngle);
            if (inputs.lockedToCage) climbMachine.fire(ClimbAction.READY_FOR_CLIMB);
        }
    }

    public class LiftingState implements PeriodicStateInterface {
        @Override
        public void periodic() {
            io.setGoalAngle(ClimbConstants.finalHangingAngle);
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
