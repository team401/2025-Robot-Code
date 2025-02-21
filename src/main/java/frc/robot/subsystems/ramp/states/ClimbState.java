package frc.robot.subsystems.ramp.states;

import frc.robot.constants.JsonConstants;

public class ClimbState extends RampState {

    @Override
    public void periodic(){
       setPosition(JsonConstants.rampConstants.climbPosition);
       super.periodic();
    }
    
 }