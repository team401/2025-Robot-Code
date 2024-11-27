package frc.robot.constants;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Units.*;

public final class ElevatorConstants {
    // TODO: Rename these
    // see ElevatorIOTalonFX
    public static final int leadElevatorMotorId = 9; // TODO: Actual CAN IDs
    public static final int followerElevatorMotorId = 10;
    public static final boolean invertFollowerElevatorMotor = false;

    public static final int elevatorCANCoder19ID = 11;
    // TODO: Tune encoder directions!
    public static final SensorDirectionValue elevatorCANCoder19Direction = SensorDirectionValue.CounterClockwise_Positive;
    public static final int elevatorCANCoder17ID = 12;
    public static final SensorDirectionValue elevatorCANCoder17Direction = SensorDirectionValue.CounterClockwise_Positive;

    // TODO: Calculate these ratios for real
    public static final double CANCoder17ToMechanismRatio = 18.0 / 17.0;
    public static final double RotorToCANCoder17Ratio = 17.0 / 18.0;

    // TODO: Tune elevator
    public static final double elevatorkP = 1.0;
    public static final double elevatorkI = 0.0;
    public static final double elevatorkD = 0.0;

    public static final double elevatorkS = 0.0;
    public static final double elevatorkV = 0.0;
    public static final double elevatorkA = 0.0;
    public static final double elevatorkG = 0.0;

    public static final LinearVelocity elevatorCruiseVelocity = MetersPerSecond.of(0.1);
    /**
     * The kV used by Motion Magic Expo to generate a motion profile.
     * Dividing the supply voltage by kV results in the maximum velocity of the system.
     * Therefore, a higher profile kV results in a lower profile velocity.
     */
    public static final Per<VoltageUnit, LinearVelocityUnit> elevatorExpo_kV = VoltsPerMeterPerSecond.ofNative(12.0);
    public static final Per<VoltageUnit, LinearAccelerationUnit> elevatorExpo_kA = VoltsPerMeterPerSecondSquared.ofNative(0.1);

    // TODO: Find actual values for these!
    public static final Distance minElevatorHeight = Meters.of(0.0);
    public static final Distance maxElevatorHeight = Meters.of(1.0);

    // TODO: Tune this value
    public static final Current elevatorStatorCurrentLimit = Amps.of(10.0);
}
