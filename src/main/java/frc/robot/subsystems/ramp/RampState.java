package frc.robot.subsystems.ramp;

import java.util.function.Consumer;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.ramp.RampSubsystem.RampTriggers;

public abstract class RampState implements PeriodicStateInterface {
    private static RampMechanism mechanism;
    protected static Consumer<RampTriggers> fireTrigger;
 
 
    //If false controls the voltage
    private static boolean positionControl = false;
    private static double controlValue = 0.0;
 
    public static void setMechanism(RampMechanism mechanism){
       RampState.mechanism = mechanism;
    }
    
    public static void setFireTrigger(Consumer<RampTriggers> fireTrigger){
       RampState.fireTrigger = fireTrigger;
    }
 
    protected void updateMechanism(){
       if (positionControl){
          mechanism.setPosition(controlValue);  
       }else{
          mechanism.setVoltage(controlValue);
       }
    }
 
    protected void setVoltage(double voltage){
       controlValue = voltage;
       positionControl = false;
    }
 
    protected void setPosition(double position){
       controlValue = position;
       positionControl = true;
    }
 
    @Override
    public void periodic(){
       updateMechanism();
    }
 
    public boolean inPosition() {
       if (positionControl){
          return Math.abs(controlValue - mechanism.inputs.position) <= JsonConstants.rampConstants.positionRange;
       }else{
          return true;
       }
    }
 
 }
 