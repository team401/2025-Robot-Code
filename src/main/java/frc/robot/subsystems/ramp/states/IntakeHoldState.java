package frc.robot.subsystems.ramp.states;

import frc.robot.constants.JsonConstants;

public class IntakeHoldState extends RampState {

    @Override
    public void periodic(){
       setVoltage(JsonConstants.rampConstants.intakeVoltage);
       super.periodic();
    }
    
 }