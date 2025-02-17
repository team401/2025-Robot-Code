package frc.robot.subsystems.ground;

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

public interface GroundIntakeIO {
  @AutoLog
  public static class GroundIntakeInputs {
    /** Current wrist position as reported by the sensor */
    public MutAngle wristPosition = Rotations.mutable(0.0);

    /** Current velocity of the wrist as reported by the sensor */
    public MutAngularVelocity wristVelocity = RotationsPerSecond.mutable(0.0);

    /** Target position for the wrist mechanism */
    public MutAngle wristGoalPosition = Rotations.mutable(0.0);

    /** Setpoint position from motion magic */
    public MutAngle wristSetpointPosition = Rotations.mutable(0.0);

    /** Target angular velocity for the wrist mechanism */
    public MutAngularVelocity wristTargetVelocity = RotationsPerSecond.mutable(0.0);

    /** Supply current of the wrist motor */
    public MutCurrent wristSupplyCurrent = Amps.mutable(0.0);

    /** Stator current of the wrist motor */
    public MutCurrent wristStatorCurrent = Amps.mutable(0.0);

    /** Velocity of the roller mechanism */
    public MutAngularVelocity rollerVelocity = RotationsPerSecond.mutable(0.0);

    /** Supply current of the roller motor */
    public MutCurrent rollerSupplyCurrent = Amps.mutable(0.0);

    /** Stator current of the roller motor */
    public MutCurrent rollerStatorCurrent = Amps.mutable(0.0);
  }

  @AutoLog
  public static class GroundIntakeOutputs {
    /** Motor output for the wrist mechanism */
    public double wristMotorOutput = 0.0;

    /** Motor output for the roller mechanism */
    public double rollerMotorOutput = 0.0;
  }

  /** Updates wrist sensor inputs */
  public default void updateWristInputs(GroundIntakeInputs inputs) {}

  /** Updates roller sensor inputs */
  public default void updateRollerInputs(GroundIntakeInputs inputs) {}

  /** Applies motor output to the wrist */
  public default void applyWristOutputs(GroundIntakeOutputs outputs) {}

  /** Applies motor output to the roller */
  public default void applyRollerOutputs(GroundIntakeOutputs outputs) {}

  /** Sets the target position for the wrist */
  public default void setWristTargetPosition(Angle goalPos) {}

  /** Sets the target speed for the roller */
  public default void setRollerTargetSpeed(double speed) {}

  /** Configures PID gains for the wrist */
  public default void configureWristPID(double kP, double kI, double kD) {}

  /** Configures PID gains for the roller */
  public default void configureRollerPID(double kP, double kI, double kD) {}

  /** Sets motion profile constraints for the wrist */
  public default void setWristMotionProfile(
      AngularVelocity maxVelocity,
      Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
      Per<VoltageUnit, AngularVelocityUnit> expo_kV) {}

  /** Sets motion profile constraints for the roller */
  public default void setRollerMotionProfile(
      AngularVelocity maxVelocity,
      Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
      Per<VoltageUnit, AngularVelocityUnit> expo_kV) {}

  /** Sets feedforward gains for the wrist */
  public default void setWristFeedForward(double kS, double kV, double kA, double kG) {}

  /** Sets feedforward gains for the roller */
  public default void setRollerFeedForward(double kS, double kV, double kA) {}

  /** Enables or disables brake mode for both motors */
  public default void enableBrakeMode(boolean brakeMode) {}

  /** Sets the current limits for both motors */
  public default void setCurrentLimits(CurrentLimitsConfigs limits) {}

  /** Enables or disables both motors */
  public default void disableMotors(boolean disabled) {}

  /** Enables or disables override mode */
  public default void enableOverrideMode(boolean override) {}

  /** Applies override current when override mode is enabled */
  public default void applyOverrideCurrent(Current current) {}
}
