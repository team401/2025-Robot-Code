package frc.robot.subsystems.ramp;

public class RampMechanism {
    RampIO io;
    State state = State.INTAKE;
    Action action = Action.INTAKE;
    RampInputsAutoLogged inputs = new RampInputsAutoLogged();
    RampOutputsAutoLogged outputs = new RampOutputsAutoLogged();

    public static enum State {
        INTAKE,
        TRANSITIONING,
        CLIMB
    }

    public static enum Action {
        INTAKE,
        CLIMB
    }

    public RampMechanism(RampIO io) {
        this.io = io;
    }

    public void periodic() {
        updateState();
        if (action == Action.INTAKE) {
            Intake();
        } else if (action == Action.CLIMB) {
            Climb();
        }
    }

    public void updateState() {
        if (inputs.position <= 0) {
            state = State.INTAKE;
        } else if (inputs.position >= 270) {
            state = State.CLIMB;
        } else {
            state = State.TRANSITIONING;
        }
    }

    public void Intake() {
        if (state == State.INTAKE) {
            outputs.appliedVolts = -0.5;
        } else {
            outputs.appliedVolts = -5.0;
        }
    }

    public void Climb() {
        if (state == State.INTAKE) {
            outputs.appliedVolts = 0.5;
        } else {
            outputs.appliedVolts = 5.0;
        }
    }

    public void setAction(Action action) {
        this.action = action;
    }

    public boolean readyForIntake() {
        return (state == State.INTAKE && action == Action.INTAKE);
    }

    public boolean inTransition() {
        return state == State.TRANSITIONING;
    }
}
