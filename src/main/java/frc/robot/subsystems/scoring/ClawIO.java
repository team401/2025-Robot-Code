package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public static class ClawInputs {
    /**
     * Whether or not the sensor detects a coral
     *
     * <p>This is left intentionally vague until we decide on CANrange vs. beam break vs. something
     * else
     */
    boolean coralDetected = false;

    /** Whether or not the sensor detects an algae */
    boolean algaeDetected = false;

    /* Supply current of the claw motor */
    MutCurrent clawSupplyCurrent = Amps.mutable(0.0);

    /* Stator current of the claw motor */
    MutCurrent clawStatorCurrent = Amps.mutable(0.0);
  }

  @AutoLog
  public static class ClawOutputs {
    /** The voltage applied to the claw motor */
    MutVoltage clawAppliedVolts = Volts.mutable(0.0);
  }

  /**
   * Updates a ClawInputs with the current information from sensors and motors.
   *
   * @param inputs The ClawInputs to update with the latest information
   */
  public default void updateInputs(ClawInputs inputs) {}

  /**
   * Applies requests to motors and updates a ClawOutputs object with information about motor
   * output.
   *
   * @param outputs ClawOutputs to update with the latest applied voltage.
   */
  public default void applyOutputs(ClawOutputs outputs) {}

  /**
   * Set the voltage to run the claw wheels
   *
   * @param volts The voltage to run the claw wheels at
   */
  public default void setVoltage(Voltage volts) {}

  /**
   * Get whether the sensor detects a coral
   *
   * @return True if a coral is detected, false if not
   */
  public default boolean getCoralDetected() {
    return false;
  }

  /**
   * Get whether the sensor detects an algae
   *
   * @return True if an algae is detected, false if not
   */
  public default boolean getAlgaeDetected() {
    return false;
  }
}
