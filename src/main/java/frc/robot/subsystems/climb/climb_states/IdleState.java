package frc.robot.subsystems.climb.climb_states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.ClimbConstants;

public class IdleState implements PeriodicStateInterface {
    
    @Override
    public void periodic() {
        io.setGoalAngle(ClimbConstants.restingAngle);
    }
    
}
