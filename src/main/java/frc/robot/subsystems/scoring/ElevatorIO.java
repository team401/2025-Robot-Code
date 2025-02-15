package frc.robot.subsystems.scoring;

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

public interface ElevatorIO {
  /**
   * What override mode the elevator is currently in
   */
  enum ElevatorOutputMode {
    ClosedLoop, // Not overriding, it should be closed loop
    Current, // Overriding, manually applying a current
    Voltage // Overriding, manually applying a voltage
  }

  @AutoLog
  public static class ElevatorInputs {
    public boolean largeEncoderConnected = false;
    public boolean smallEncoderConnected = false;

    /** The angle of the 19 tooth-gear-encoder. Counts total rotations, not absolute position. */
    public MutAngle largeEncoderPos = Rotations.mutable(0.0);

    /** Current velocity of the 19 tooth-gear-encoder. */
    public MutAngularVelocity largeEncoderVel = RotationsPerSecond.mutable(0.0);

    /** The angle of the 17 tooth-gear-encoder. Counts total rotations, not absolute position. */
    public MutAngle smallEncoderPos = Rotations.mutable(0.0);

    /** Absolute position of the 19 tooth-gear-encoder. Wraps around after 1 rotation */
    public MutAngle largeEncoderAbsolutePos = Rotations.mutable(0.0);

    public double largeAbsPosRot = 0.0;

    /** Absolute position of the 17 tooth-gear-encoder. Wraps around after 1 rotation */
    public MutAngle smallEncoderAbsolutePos = Rotations.mutable(0.0);

    /** Goal position of the elevator */
    public MutAngle largeEncoderGoalPos = Rotations.mutable(0.0);

    /** Profile setpoint goal position of the elevator */
    public MutAngle largeEncoderSetpointPos = Rotations.mutable(0.0);

    /** Stator current of the lead elevator motor */
    public MutCurrent elevatorLeadMotorStatorCurrent = Amps.mutable(0.0);

    /** Supply current of the lead elevator motor */
    public MutCurrent elevatorLeadMotorSupplyCurrent = Amps.mutable(0.0);

    /** Stator current of the follower elevator motor */
    public MutCurrent elevatorFollowerMotorStatorCurrent = Amps.mutable(0.0);

    /** Supply current of the follower elevator motor */
    public MutCurrent elevatorFollowerMotorSupplyCurrent = Amps.mutable(0.0);

    /**
     * Current closed-loop error (distance from setpoint position) as reported by the lead motor
     * TalonFX, in rotations.
     */
    public double motionMagicError = 0.0;

    /** Velocity of the mechanism, as reported by the lead motor TalonFX */
    public MutAngularVelocity elevatorMechanismVelocity = RotationsPerSecond.mutable(0.0);
  }

  @AutoLog
  public static class ElevatorOutputs {
    /** Are the elevator motors currently disabled in software? */
    public boolean motorsDisabled = false;

    /** The current output mode of the elevator */
    public ElevatorOutputMode outputMode = ElevatorOutputMode.ClosedLoop;

    /** The voltage applied to the elevator motor */
    public MutVoltage elevatorAppliedVolts = Volts.mutable(0.0);

    /** Contribution of the p-term to motor output */
    public MutVoltage pContrib = Volts.mutable(0.0);

    /** Contribution of the i-term to motor output */
    public MutVoltage iContrib = Volts.mutable(0.0);

    /** Contribution of the d-term to motor output */
    public MutVoltage dContrib = Volts.mutable(0.0);
  }

  /**
   * Updates an ElevatorInputs with the current information from sensors readings and from the
   * motors.
   *
   * @param inputs ElevatorInputs object to update with latest information
   */
  public void updateInputs(ElevatorInputs inputs);

  /**
   * Applies requests to motors and updates an ElevatorOutputs object with information about motor
   * output.
   *
   * @param outputs ElevatorOutputs update with latest applied voltage
   */
  public void applyOutputs(ElevatorOutputs outputs);

  /**
   * Set the goal position of CANCoder 19 which the elevator will control to when it is not in
   * override mode
   *
   * <p>This method should only be called by the ElevatorMechanism! There is important safety
   * control logic housed there which, if bypassed, will be sorely missed.
   */
  public void setLargeCANCoderGoalPos(Angle goalPos);

  /** Get the absolute position of the 19 tooth CANCoder. */
  public Angle getLargeCANCoderAbsPos();

  /** Get the absolute position of the 17 tooth CANCoder. */
  public Angle getSmallCANCoderAbsPos();

  /**
   * Set the position of the 19 tooth CANCoder. This position is separate from absolute position and
   * can track multiple rotations.
   */
  public void setLargeCANCoderPosition(Angle newAngle);

  /**
   * Set the position of the 17 tooth CANCoder. This position is separate from absolute position and
   * can track multiple rotations.
   */
  public void setSmallCANCoderPosition(Angle newAngle);

  /**
   * Set the override voltage for the elevator when in Voltage output mode
   * 
   * @param volts The voltage to apply
   */
  public void setOverrideVoltage(Voltage volts);

  /**
   * Set the static current (because of FOC) that will be applied when the elevator is in Current output
   * mode.
   */
  public void setOverrideCurrent(Current current);

  /**
   * Set whether the elevator should use ClosedLoop control (default), voltage override, or current override
   */
  public void setOutputMode(ElevatorOutputMode mode);

  /** Update PID gains for the elevator */
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

  /** Set the stator current limit for both elevator motors */
  public void setStatorCurrentLimit(Current currentLimit);

  /** Set whether or not the motors on the elevator should be disabled. */
  public void setMotorsDisabled(boolean disabled);
}
