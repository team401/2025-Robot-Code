package frc.robot.subsystems.ground;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
  @AutoLog
  public static class GroundIntakeInputs {
    
    MutAngle shoulderMotorPos = Rotations.mutable(0.0);
    MutAngle shoulderMotorGoalPos = Rotations.mutable(0.0);
    MutCurrent shoulderMotorSupplyCurrent = Amps.mutable(0.0);
    MutCurrent shoulderMotorStatorCurrent = Amps.mutable(0.0);
    MutCurrent rollerMotorSupplyCurrent = Amps.mutable(0.0);
    MutCurrent rollerMotorStatorCurrent = Amps.mutable(0.0);
  }

  @AutoLog
  public static class GroundIntakeOutputs {
    /** The voltage applied to the claw motor */
    MutVoltage rollerAppliedVolts = Volts.mutable(0.0);
    MutCurrent shoulderAppliedCurrent = Amps.mutable(0.0);
  }

  /**
   * Updates a ClawInputs with the current information from sensors and motors.
   *
   * <p>Should be called by the ClawMechanism periodically
   *
   * @param inputs The ClawInputs to update with the latest information
   */
  public default void updateInputs(GroundIntakeInputs inputs) {}

  /**
   * Applies requests to motors and updates a ClawOutputs object with information about motor
   * output.
   *
   * <p>Should be called by the ClawMechanism periodically
   *
   * @param outputs ClawOutputs to update with the latest applied voltage.
   */
  public default void applyOutputs(GroundIntakeOutputs outputs) {}

  /**
   * Set the voltage to run the claw wheels
   *
   * @param volts The voltage to run the claw wheels at
   */
  public default void setRollerVoltage(Voltage volts) {}
  public default void setShoulderTunerCurrent(Current amps) {}


  public default void setShoulderGoalPosition(Angle goalAngle){}

  /**
   * Get the current position of the claw motor
   *
   * @return An Angle, the current position of the roller motor as reported by the TalonFX
   */
  public default Angle getGroundIntakePos() {
    return Rotations.zero();
  }

}
