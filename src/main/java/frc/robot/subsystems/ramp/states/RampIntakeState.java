package frc.robot.subsystems.ramp.states;

import frc.robot.constants.JsonConstants;

public class RampIntakeState extends RampState {

    @Override
    public void periodic(){
       setPosition(JsonConstants.rampConstants.intakePosition);
       if (inPosition()){
          fireTrigger.accept(RampTriggers.HOLD_INTAKE);
       }
       super.periodic();
    }
    
 }
