package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.Per;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristInputs {
    /** Current wrist position as reported by the CANcoder */
    public MutAngle wristPosition = Rotations.mutable(0.0);

    /** Current velocity of the wrist as reported by the CANcoder */
    public MutAngularVelocity wristVelocity = RotationsPerSecond.mutable(0.0);

    /** Goal angle of the wrist, as seen by the wrist CANcoder */
    public MutAngle wristGoalPosition = Rotations.mutable(0.0);

    /** The setpoint target position from Motion Magic Expo */
    public MutAngle wristSetpointPosition = Rotations.mutable(0.0);

    /** The target angular velocity of the wrist */
    public MutAngularVelocity wristTargetVelocity = RotationsPerSecond.mutable(0.0);

    /** Supply current of the wrist motor */
    public MutCurrent wristSupplyCurrent = Amps.mutable(0.0);

    /** Stator current of the wrist motor */
    public MutCurrent wristStatorCurrent = Amps.mutable(0.0);
  }

  @AutoLog
  public static class WristOutputs {
    /**
     * The closed-loop output of the wrist controller. This value isn't a unit because phoenix 6
     * doesn't use a unit for this value (as it can be a Voltage or a Current depending on whether
     * or not FOC is used).
     */
    public double wristOutput = 0.0;
  }

  /**
   * Updates a WristInputs with the current information from sensors and motors.
   *
   * <p>Should be called by the WristMechanism periodically
   *
   * @param inputs The WristInputs to update with the latest information
   */
  public default void updateInputs(WristInputs inputs) {}

  /**
   * Applies requests to motors and updates a WristOutputs object with information about motor
   * output.
   *
   * <p>Should be called by the WristMechanism periodically
   *
   * @param outputs WristOutputs to update with the latest applied voltage.
   */
  public default void applyOutputs(WristOutputs outputs) {}

  /**
   * Set the wrist's goal position.
   *
   * <p>Should be called by the WristMechanism
   *
   * @param goalPos Goal position of the wrist, as seen by the wrist CANcoder
   */
  public default void setWristGoalPos(Angle goalPos) {}

  /** Update PID gains for wrist */
  public default void setPID(double kP, double kI, double kD) {}

  /** Set profile constraints to be sent to Motion Magic Expo */
  public default void setMaxProfile(
      AngularVelocity maxVelocity,
      Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
      Per<VoltageUnit, AngularVelocityUnit> expo_kV) {}

  /** Set feedforward gains for closed-loop control */
  public default void setFF(double kS, double kV, double kA, double kG) {}

  /** Set whether or not the motors should brake while idle */
  public default void setBrakeMode(boolean brakeMode) {}

  /** Set the current limits for the wrist motor */
  public default void setCurrentLimits(CurrentLimitsConfigs limits) {}

  /** Set whether or not the wrist motor should be disabled. */
  public default void setMotorsDisabled(boolean disabled) {}

  /**
   * Sets whether or not the wrist is in 'override' mode
   *
   * @param override True if override, false if normal. This value defaults/is initialized to false
   *     until changed
   */
  public default void setOverrideMode(boolean override) {}

  /**
   * Set the current to apply in override mode.
   *
   * <p>This is a current because the wrist uses TorqueCurrentFOC for control instead of voltage.
   *
   * @param current The current to apply. This will only be applied after setOverrideMode(true) is
   *     called.
   */
  public default void setOverrideCurrent(Current current) {}
}
