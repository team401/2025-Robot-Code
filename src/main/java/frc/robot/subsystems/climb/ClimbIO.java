package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  @AutoLog
  public static class ClimbInputs {
    boolean lockedToCage = false;

    MutAngle motorAngle = Radians.mutable(0);
    MutAngle goalAngle = Radians.mutable(0);
  }

  @AutoLog
  public static class ClimbOutputs {
    MutVoltage appliedVoltage = Volts.mutable(0);
    // MutAngle goalAngle = Radians.mutable(0);
  }

  public default void updateInputs(ClimbInputs inputs) {}

  public default void applyOutputs(ClimbOutputs outputs) {}

  public default void setGoalAngle(Angle angle) {}

  public default void setOverrideVoltage(Voltage voltage) {}

  public default void setBrakeMode(boolean brake) {}

  public default void setPID(double p, double i, double d) {}

  public default void setFF(double kS, double kV, double kA, double kG) {}
}
