package frc.robot.subsystems.ground;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
  @AutoLog
  public static class GroundIntake {
    /**
     * Whether or not the sensor detects a coral
     *
     * <p>This is left intentionally vague until we decide on CANrange vs. beam break vs. something
     * else
     */
    boolean coralDetected = false;

    /** Whether or not the sensor detects an algae */
    boolean algaeDetected = false;

    MutAngle clawMotorPos = Rotations.mutable(0.0);

    /* Supply current of the claw motor */
    MutCurrent clawSupplyCurrent = Amps.mutable(0.0);

    /* Stator current of the claw motor */
    MutCurrent clawStatorCurrent = Amps.mutable(0.0);
  }

  @AutoLog
  public static class GroundIntakeOutputs {
    /** The voltage applied to the claw motor */
    MutVoltage clawAppliedVolts = Volts.mutable(0.0);
  }

  /**
   * Updates a ClawInputs with the current information from sensors and motors.
   *
   * <p>Should be called by the ClawMechanism periodically
   *
   * @param inputs The ClawInputs to update with the latest information
   */
  public default void updateInputs(GroundIntake inputs) {}

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
  public default void setVoltage(Voltage volts) {}

  /**
   * Get the current position of the claw motor
   *
   * @return An Angle, the current position of the roller motor as reported by the TalonFX
   */
  public default Angle getGroundIntakePos() {
    return Rotations.zero();
  }

}
