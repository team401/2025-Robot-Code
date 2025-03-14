package frc.robot.subsystems.ramp.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.ramp.RampMechanism;
import java.util.function.Consumer;

public abstract class RampState implements PeriodicStateInterface {

  public enum RampTriggers {
    RETURN_TO_IDLE,
    INTAKE,
    CLIMB,
    HOME,
    HOMING_FINISHED
  }

  protected static RampMechanism mechanism;
  public static Consumer<RampTriggers> fireTrigger;

  // If false controls the voltage
  private static boolean positionControl = false;
  private static double controlValue = 0.0;

  public static void setMechanism(RampMechanism mechanism) {
    RampState.mechanism = mechanism;
  }

  public static void setFireTrigger(Consumer<RampTriggers> fireTrigger) {
    RampState.fireTrigger = fireTrigger;
  }

  protected void updateMechanism() {
    if (positionControl) {
      mechanism.setPosition(controlValue);
    } else {
      mechanism.setVoltage(controlValue);
    }
  }

  protected void setVoltage(double voltage) {
    controlValue = voltage;
    positionControl = false;
  }

  protected void setPosition(double position) {
    controlValue = position;
    positionControl = true;
  }

  public void periodic() {
    updateMechanism();
  }

  public boolean inPosition() {
    if (positionControl) {
      return Math.abs(controlValue - mechanism.inputs.position)
          <= JsonConstants.rampConstants.positionRange;
    } else {
      return true;
    }
  }
}
