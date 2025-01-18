package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;

import com.google.common.base.Supplier;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
 
    private ClimbIO io;
    private ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();
    private ClimbOutputsAutoLogged outputs = new ClimbOutputsAutoLogged();

    private ClimbAction Action = ClimbAction.WAIT;
    private ClimbState State = ClimbState.IDLE;
    
    private Supplier<Boolean> rampClear = () -> true;
    
    private enum ClimbAction {
        WAIT, // do nothing
        LOCKING, // move arm up until sensors triggered
        LIFTING, // move arm down to hang
        OVERRIDE
    }

    public enum ClimbState {
        IDLE, // do nothing
        WAITING, // waiting for system to be ready for climb
        SEARCHING, // searching for target
        LIFTING, // moving arm to hang
        HANGING, // hanging from bars steadily
        OVERRIDE
    }

    public ClimbSubsystem(ClimbIO io) {
        this.io = io;
    }

    public void setAction(ClimbAction action) {
        Action = action;
    }

    //todo fix
    private boolean reachedAngle() {
        return inputs.motorAngle.in(Degrees) > 90;
    }

    private void idle() {
        io.setVoltage(0.0);

        if (Action == ClimbAction.LOCKING) {
            if (!rampClear.get()) State = ClimbState.WAITING;
            else if (!inputs.lockedToCage) State = ClimbState.SEARCHING;
        } else if (Action == ClimbAction.LIFTING) {
            if (!reachedAngle()) State = ClimbState.LIFTING;
            else State = ClimbState.HANGING;
        } else if (Action == ClimbAction.OVERRIDE) {
            State = ClimbState.OVERRIDE;
        }
    }

}
