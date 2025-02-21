package frc.robot.subsystems.ramp.states;

import coppercore.controls.state_machine.transition.Transition;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.ramp.RampSubsystem.RampStates;

public class IntakeHoldState extends RampState {

   @Override
   public void periodic(){
      setVoltage(JsonConstants.rampConstants.intakeVoltage);
      super.periodic();
   }

   @Override
   public void onExit(@SuppressWarnings("rawtypes") Transition transition){
      mechanism.setHome();
   }
    
 }