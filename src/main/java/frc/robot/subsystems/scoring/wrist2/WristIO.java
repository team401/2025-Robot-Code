package frc.robot.subsystems.scoring.wrist2;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  enum WristOutputMode {
    ClosedLoop, // Not overriding, it should be closed loop
    Current, // Overriding, manually applying a current
    Voltage // Overriding, manually applying a voltage
  }

  @AutoLog
  public static class WristInputs {
    public boolean wristMotorConnected = false;

    /** Stator current of the wristMotor */
    public MutCurrent wristMotorStatorCurrent = Amps.mutable(0.0);

    /** Supply current of the wristMotor */
    public MutCurrent wristMotorSupplyCurrent = Amps.mutable(0.0);
    

    public boolean wristEncoderConnected = false;

    /** Current position of the wristEncoder. This measures total rotation since power on, not absolute position */
    public MutAngle wristEncoderPos = Rotations.mutable(0.0);

    /** Current velocity reported by the wristEncoder */
    public MutAngularVelocity wristEncoderVel = RotationsPerSecond.mutable(0.0);

    
    /** The current closed-loop goal position of the name, in terms of the wristEncoder */
    public MutAngle wristEncoderGoalPos = Rotations.mutable(0.0);

    /** Profile setpoint goal position of the name, in terms of the wristEncoder */
    public MutAngle wristEncoderSetpointPos = Rotations.mutable(0.0);

    /**
     * Current closed-loop error (distance from setpoint position) as reported by the wristMotor
     * TalonFX, in rotations.
     */
    public double motionMagicError = 0.0;

    /** Velocity of the Wrist mechanism, as reported by the wristMotor TalonFX */
    public MutAngularVelocity wristVelocity = RotationsPerSecond.mutable(0.0);
  }

  @AutoLog
  public static class WristOutputs {
    /** Are the motors currently disabled in software? */
    public boolean motorsDisabled = false;

    /** The current output mode of the Wrist */
    public WristOutputMode outputMode = WristOutputMode.ClosedLoop;

    /** The voltage currently applied to the motors */
    public MutVoltage wristAppliedVolts = Volts.mutable(0.0);

    /** The current closed-loop output from Motion Magic */
    public double wristClosedLoopOutput = 0.0;

    /** Contribution of the p-term to motor output */
    public MutVoltage pContrib = Volts.mutable(0.0);

    /** Contribution of the i-term to motor output */
    public MutVoltage iContrib = Volts.mutable(0.0);

    /** Contribution of the d-term to motor output */
    public MutVoltage dContrib = Volts.mutable(0.0);
  }

  /**
   * Updates a WristInputs with the current information from sensors readings and from the
   * motors.
   *
   * @param inputs WristInputs object to update with latest information
   */
  public void updateInputs(WristInputs inputs);

  /**
   * Applies requests to motors and updates a WristOutputs object with information about motor
   * output.
   *
   * @param outputs WristOutputs object to update with latest applied outputs
   */
  public void applyOutputs(WristOutputs outputs);

  /**
   * Set the goal position of wristEncoder which the Wrist will control to when it is not in
   * override mode
   */
  public void setWristEncoderGoalPos(Angle goalPos);

  /**
   * Set the position of the wristEncoder. This position is separate from absolute position and
   * can track multiple rotations.
   */
  public void setWristEncoderPosition(Angle newAngle);

  /**
   * Set the override voltage for the Wrist when in Voltage output mode
   *
   * @param volts The voltage to apply
   */
  public void setOverrideVoltage(Voltage volts);

  /**
   * Set the static current (because of FOC) that will be applied when the Wrist is in Current
   * output mode.
   */
  public void setOverrideCurrent(Current current);

  /**
   * Set whether the Wrist should use ClosedLoop control (default), voltage override, or current
   * override
   */
  public void setOutputMode(WristOutputMode mode);

  /** Update PID gains for the Wrist */
  public void setPID(double p, double i, double d);

  /** Set profile constraints to be sent to Motion Magic Expo */
  public void setMaxProfile(
      AngularVelocity maxVelocity,
      Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
      Per<VoltageUnit, AngularVelocityUnit> expo_kV);

  /** Set feedforward gains for closed-loop control */
  public void setFF(double kS, double kV, double kA, double kG);

  /** Set whether or not the motors should brake while idle */
  public void setBrakeMode(boolean brakeMode);

  /** Set the stator current limit for the Wrist motor */
  public void setStatorCurrentLimit(Current currentLimit);

  /** Set whether or not the motor on the Wrist should be disabled. */
  public void setMotorsDisabled(boolean disabled);
}