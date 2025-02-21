package frc.robot.subsystems.ramp;

import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.ramp.RampSubsystem.RampTriggers;

public class IntakeHoldState extends RampState {

    @Override
    public void periodic(){
       setVoltage(JsonConstants.rampConstants.intakeVoltage);
       super.periodic();
    }
    
 }