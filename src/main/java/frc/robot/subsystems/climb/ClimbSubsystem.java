package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

    private ClimbIO io;
    private ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();
    private ClimbOutputsAutoLogged outputs = new ClimbOutputsAutoLogged();

    private ClimbAction Action = ClimbAction.NONE;
    private ClimbState State = ClimbState.IDLE;

    private BooleanSupplier rampClear = () -> true;

    public enum ClimbAction {
        NONE, // do nothing
        CLIMB, // start automated climb sequence
        OVERRIDE
    }

    private enum ClimbState {
        IDLE, // do nothing
        WAITING, // waiting for system to be ready for climb
        SEARCHING, // searching for target
        LIFTING, // moving arm to hang
        OVERRIDE
    }

    public ClimbSubsystem(ClimbIO io) {
        this.io = io;
    }

    public void setAction(ClimbAction action) {
        Action = action;
    }

    private void idle() {
        io.setGoalAngle(ClimbConstants.restingAngle);

        if (Action == ClimbAction.CLIMB) {
            State = ClimbState.WAITING;
        } else if (Action == ClimbAction.OVERRIDE) {
            State = ClimbState.OVERRIDE;
        }
    }

    private void waiting() {
        if (Action != ClimbAction.CLIMB) {
            State = ClimbState.IDLE;
        } else if (rampClear.getAsBoolean()) {
            State = ClimbState.SEARCHING;
        } else {
            io.setGoalAngle(ClimbConstants.restingAngle);
        }
    }

    private void searching() {
        if (Action != ClimbAction.CLIMB) {
            State = ClimbState.IDLE;
        } else if (!rampClear.getAsBoolean()) {
            State = ClimbState.WAITING;
        } else if (inputs.lockedToCage) {
            State = ClimbState.LIFTING;
        } else {
            io.setGoalAngle(ClimbConstants.searchingAngle);
            // passive lock to bars
            // drive forward?
        }
    }

    private void lifting() {
        if (Action != ClimbAction.CLIMB) {
            State = ClimbState.IDLE;
        } else if (!rampClear.getAsBoolean() || !inputs.lockedToCage) {
            State = ClimbState.WAITING;
        } else if (inputs.motorAngle.in(Degrees) > 20) {
            io.setGoalAngle(ClimbConstants.finalHangingAngle);
            // passive lock to bars
        }
    }

    private void override() {
        if (Action != ClimbAction.OVERRIDE) {
            State = ClimbState.IDLE;
        } else {
            // idk not my top priority tbh
        }
    }

    @Override
    public void periodic() {

        io.updateInputs(inputs);
        io.applyOutputs(outputs);

        switch (State) {
            case WAITING:
                waiting();
                break;
            case SEARCHING:
                searching();
                break;
            case LIFTING:
                lifting();
                break;
            default:
                idle();
                break;
        }

        Logger.processInputs("climb/inputs", inputs);
        Logger.processInputs("climb/outputs", outputs);
        Logger.recordOutput("climb/State", State);
        Logger.recordOutput("climb/Action", Action);
    }
}
