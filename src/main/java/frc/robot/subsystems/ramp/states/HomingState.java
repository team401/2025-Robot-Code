package frc.robot.subsystems.ramp.states;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.JsonConstants;

public class HomingState extends RampState {

   @Override
   public void periodic(){
      //Goto hard stop

      if (/* At hard stop detection */){
         mechanism.setHome();
         fireTrigger.accept(RampTriggers.HOMED);
      }

      super.periodic();
   }
    
 }