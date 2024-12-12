package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Per;

public final class ElevatorConstants {
    // TODO: Rename these
    // see ElevatorIOTalonFX
    public static final int leadElevatorMotorId = 9; // TODO: Actual CAN IDs
    public static final int followerElevatorMotorId = 10;
    public static final boolean invertFollowerElevatorMotor = false;

    public static final int smallCANCoderTeeth = 17;
    public static final int largeCANCoderTeeth = 19;
    public static final int spoolTeeth = 18;

    /** 5 motor rotations to 1 spool rotation */
    public static final double elevatorReduction = 5.0;

    // TODO: Actual values
    public static final Mass carriageMass = Pounds.of(20);
    public static final Distance drumRadius = Inches.of(1.503);

    // TODO: Tune encoder directions!
    public static final int elevatorLargeCANCoderID = 11;
    public static final SensorDirectionValue elevatorLargeCANCoderDirection =
            SensorDirectionValue.CounterClockwise_Positive;
    public static final int elevatorSmallCANCoderID = 12;
    public static final SensorDirectionValue elevatorSmallCANCoderDirection =
            SensorDirectionValue.CounterClockwise_Positive;

    // TODO: Calculate these ratios for real
    public static final double largeCANCoderToMechanismRatio = 1.0;
    //     (double) largeCANCoderTeeth / (double) spoolTeeth;
    public static final double rotorToLargeCANCoderRatio =
            elevatorReduction * (double) largeCANCoderTeeth / (double) spoolTeeth;

    // TODO: Tune elevator
    public static final double elevatorkP = 20.0;
    public static final double elevatorkI = 0.2;
    public static final double elevatorkD = 0.0;

    public static final double elevatorkS = 0.0;
    public static final double elevatorkV = 0.0;
    public static final double elevatorkA = 0.0;
    public static final double elevatorkG = 0.0;

    // TODO: Actual ratios
    public static final Per<DistanceUnit, AngleUnit> elevatorToSpool =
            Inches.of(4.724).divide(Rotations.of(1));

    public static final LinearVelocity elevatorCruiseVelocity = MetersPerSecond.of(0.1);
    // TODO: Factor in gearbox ratios and actual calculations into this constant
    public static final AngularVelocity elevatorAngularCruiseVelocity =
            RadiansPerSecond.of(
                    elevatorCruiseVelocity.in(MetersPerSecond)
                            / elevatorToSpool.in(PerUnit.combine(Meters, Radians)));

    /**
     * The kV used by Motion Magic Expo to generate a motion profile. Dividing the supply voltage by
     * kV results in the maximum velocity of the system. Therefore, a higher profile kV results in a
     * lower profile velocity.
     */
    public static final Per<VoltageUnit, AngularVelocityUnit> elevatorExpo_kV =
            VoltsPerRadianPerSecond.ofNative(1.0);

    public static final Per<VoltageUnit, AngularAccelerationUnit> elevatorExpo_kA =
            VoltsPerRadianPerSecondSquared.ofNative(0.2);

    // TODO: Find actual values for these!
    public static final Distance minElevatorHeight = Meters.of(0.0);
    public static final Distance maxElevatorHeight = Meters.of(3.0);

    // TODO: Tune this value
    public static final Current elevatorStatorCurrentLimit = Amps.of(10.0);

    public static final int CRTticksPerRotation =
            4096; // CANCoders have a resolution of 4096 ticks/rotation
}
