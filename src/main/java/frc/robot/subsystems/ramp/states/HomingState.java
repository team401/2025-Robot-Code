package frc.robot.subsystems.ramp.states;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.JsonConstants;
import org.littletonrobotics.junction.Logger;

public class HomingState extends RampState {

  private boolean hasMoved = false;
  private final Timer homingTimer = new Timer();

  private final MedianFilter velocityFilter =
      new MedianFilter(JsonConstants.rampConstants.homingVelocityFilterWindowSize);

  @Override
  public void onEntry(@SuppressWarnings("rawtypes") Transition transition) {
    hasMoved = false;
    homingTimer.reset();
    homingTimer.start();
  }

  @Override
  public void periodic() {
    if (!DriverStation.isEnabled()) {
      homingTimer.restart();
      return;
    }

    setVoltage(JsonConstants.rampConstants.homingVoltage);

    if (hasMoved
        && homingTimer.hasElapsed(
            JsonConstants.elevatorConstants.homingMaxUnmovingTime.in(Seconds))) {
      mechanism.setHome();
      fireTrigger.accept(RampTriggers.HOMING_FINISHED);
    }

    double filteredAbsVelocity =
        velocityFilter.calculate(mechanism.getVelocity().abs(RadiansPerSecond));

    Logger.recordOutput("ramp/homing/filteredAbsVelocity", filteredAbsVelocity);
    Logger.recordOutput("ramp/homing/hasMoved", hasMoved);
    Logger.recordOutput("ramp/homing/homingTimer", homingTimer.get());

    if (filteredAbsVelocity
        < JsonConstants.rampConstants.homingVelocityThresholdMetersPerSecond.in(RadiansPerSecond)) {
      if (hasMoved) {
        mechanism.setHome();
        fireTrigger.accept(RampTriggers.HOMING_FINISHED);
      } else if (homingTimer.hasElapsed(
          JsonConstants.rampConstants.homingMaxUnmovingTime.in(Seconds))) {
        mechanism.setHome();
        fireTrigger.accept(RampTriggers.HOMING_FINISHED);
      }
    } else {
      hasMoved = true;
    }

    if (homingTimer.hasElapsed(JsonConstants.rampConstants.homingMaxTime.in(Seconds))) {
      mechanism.setHome();
      fireTrigger.accept(RampTriggers.HOMING_FINISHED);
    }

    super.periodic();
  }

  public boolean inPosition() {
    return false;
  }
}
