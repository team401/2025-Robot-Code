package frc.robot.subsystems.ramp;

import org.littletonrobotics.junction.AutoLog;

public interface RampIO {

  @AutoLog
  public static class RampInputs {
    public double position = 0.0;
    public boolean inPosition;
  }

  @AutoLog
  public static class RampOutputs {
    public double targetPosition;
    public double appliedVolts;
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

  public void updateInputs(RampInputs inputs);

  public void updateOutputs(RampInputs inputs, RampOutputs outputs);
}
