package frc.robot.subsystems.ramp;

import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.ramp.RampSubsystem.RampTriggers;

public class IntakeState extends RampState {

    @Override
    public void periodic(){
       setPosition(JsonConstants.rampConstants.intakePosition);
        if (inPosition()){
            fireTrigger.accept(RampTriggers.HOLD_INTAKE);
        }
        super.periodic();
    }
    
 }
