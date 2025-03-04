package frc.robot.subsystems.ramp;

import org.littletonrobotics.junction.AutoLog;

public interface RampIO {

  @AutoLog
  public static class RampInputs {
    public double position = 0.0;
    public boolean positionControl = false;
    public double controlValue = 0.0;
  }

  @AutoLog
  public static class RampOutputs {
    public double appliedVolts;
    public double velocity;
  }

  /**
   * Set the PID gains used for closed-loop control
   *
   * <p>This should be used in test mode for tuning
   *
   * @param kP
   * @param kI
   * @param kD
   */
  public void setPID(double kP, double kI, double kD);

  public void addOffset(double offset);

  public void updateInputs(RampInputs inputs);

  public void updateOutputs(RampInputs inputs, RampOutputs outputs);
}
