package frc.robot.constants.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;

public final class ElevatorConstants {
  @JSONExclude
  public static final JSONSync<ElevatorConstants> synced =
      new JSONSync<ElevatorConstants>(
          new ElevatorConstants(),
          "ElevatorConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Integer leadElevatorMotorId = 12;
  public final Integer followerElevatorMotorId = 11;
  public final Boolean invertFollowerElevatorMotor = true;

  public final Integer smallCANCoderTeeth = 17;
  public final Integer largeCANCoderTeeth = 19;
  public final Integer spoolTeeth = 18;

  /** 5 motor rotations to 1 spool rotation */
  public final Double elevatorReduction = 5.0;

  // TODO: Actual values

  public final Mass carriageMass = Pounds.of(20.0);

  public final Distance drumRadius = Inches.of(0.7515); // 1.503 inch diameter / 2

  // TODO: Use coppercore gear math after
  // https://github.com/team401/coppercore/issues/52 is
  // done.
  public final Distance elevatorHeightPerSpoolRotation = Inches.of(4.724);

  /**
   * What point in the sensor's range the discontinuity occurs. Results in a range of [1-x, x). For
   * example, a value of 1 gives a range of [0.0, 1). This is what we want to use for CRT elevator
   * sensors.
   */
  public final Double elevatorCANCoderDiscontinuityPoint = 1.0;

  // TODO: Tune encoder directions!
  public final Integer elevatorLargeCANCoderID = 14;

  public final SensorDirectionValue elevatorLargeCANCoderDirection =
      SensorDirectionValue.Clockwise_Positive;

  public final Integer elevatorSmallCANCoderID = 15;

  public final SensorDirectionValue elevatorSmallCANCoderDirection =
      SensorDirectionValue.CounterClockwise_Positive;

  /*
   * The large CANCoder is represented as the mechanism in our Phoenix configs.
   * This means that we are controlling to a goal in terms of large CANCoder angle.
   */
  @JSONExclude public final double largeCANCoderToMechanismRatio = 1.0;

  @JSONExclude
  public final double rotorToLargeCANCoderRatio =
      elevatorReduction * (double) largeCANCoderTeeth / (double) spoolTeeth;

  // TODO: Tune elevator
  public final Double elevatorkP = 60.0;
  public final Double elevatorkI = 0.0;
  public final Double elevatorkD = 10.0;

  public final Double elevatorkS = 0.0;
  public final Double elevatorkV = 2.0;
  public final Double elevatorkA = 0.08;
  public final Double elevatorkG = 16.0;

  // TODO: Actual ratios
  @JSONExclude
  public final Per<DistanceUnit, AngleUnit> elevatorToSpool =
      elevatorHeightPerSpoolRotation.div(Rotations.of(1));

  public final Double elevatorCruiseVelocityMetersPerSecond = 3.0;

  @JSONExclude
  public final LinearVelocity elevatorCruiseVelocity =
      MetersPerSecond.of(elevatorCruiseVelocityMetersPerSecond);

  // TODO: Factor in gearbox ratios and actual calculations into this constant
  @JSONExclude
  public final AngularVelocity elevatorAngularCruiseVelocity =
      RadiansPerSecond.of(
          elevatorCruiseVelocity.in(MetersPerSecond)
              / elevatorToSpool.in(PerUnit.combine(Meters, Radians)));

  /* The Motion Magic Expo kV, measured in Volts per Radian per Second, but represented as a double so it can be synced by JSONSync */
  public final Double elevatorExpo_kV_raw = 0.1;

  /**
   * The kV used by Motion Magic Expo to generate a motion profile. Dividing the supply voltage by
   * kV results in the maximum velocity of the system. Therefore, a higher profile kV results in a
   * lower profile velocity.
   */
  @JSONExclude
  public final Per<VoltageUnit, AngularVelocityUnit> elevatorExpo_kV =
      VoltsPerRadianPerSecond.ofNative(elevatorExpo_kV_raw);

  /* The Motion Magic Expo kA, measured in Volts per Radian per Second Squared, but represented as a double so it can be synced by JSONSync */
  public final Double elevatorExpo_kA_raw = 0.01;

  @JSONExclude
  public final Per<VoltageUnit, AngularAccelerationUnit> elevatorExpo_kA =
      VoltsPerRadianPerSecondSquared.ofNative(elevatorExpo_kA_raw);

  public final Distance minElevatorHeight = Meters.of(0.0);
  public final Distance maxElevatorHeight = Meters.of(1.9);

  // TODO: Tune this value
  public final Current elevatorStatorCurrentLimit = Amps.of(80.0);

  public final Integer CRTticksPerRotation =
      4096; // CANCoders have a resolution of 4096 ticks/rotation

  /**
   * How many meters away from setpoint the elevator height can be while still being considered "at
   * the setpoint"
   */
  public final Distance elevatorTargetThresholdMeters = Meters.of(0.03);

  public final Integer medianFilterWindowSize = 10; // We might want to go smaller with this value

  public final Boolean ignoreCRT = true;

  public final Voltage homingVoltage = Volts.of(-3);

  /**
   * What reported velocity should be considered "moving" while homing, stored as a Double because
   * we can't serialize a MetersPerSecond
   */
  public final Double homingVelocityThresholdMetersPerSecond = 0.1;

  /** The maximum amount of time the elevator can home for before saying it's at 0 and giving up */
  public final Time homingMaxTime = Seconds.of(3.0);

  /**
   * The maximum amount of time the elevator can home without ever moving before it knows its at 0
   */
  public final Time homingMaxUnmovingTime = Seconds.of(0.25);

  public final Integer homingVelocityFilterWindowSize = 5;

  /**
   * This is the minimum height the elevator can reach while the wrist is down without hitting it on
   * the chassis below
   */
  public final Distance minWristDownHeight = Meters.of(0.2); // TODO: Real value

  /** Minimum height the elevator can be at with the wrist out without hitting the reef base */
  public final Distance minReefSafeHeight = Meters.of(0.6);

  public final Distance minWristDownReefBaseSafeHeight = Meters.of(0.55);

  public final Distance L4MinHeightAbove = Meters.of(1.74);

  public final Distance L3MinHeightAbove = Meters.of(0.9);

  public final Distance L2MaxHeightBelow = Meters.of(0.16);

  /** When within this distance, the elevator is considered "at its setpoint" */
  public final Distance elevatorSetpointEpsilon = Meters.of(0.05);

  /** When within this distance, spin the rollers during net pop shot */
  public final Distance netScoreSetpointEpsilon = Meters.of(0.25);

  /**
   * How high elevator must go before we can swing the algae up into the robot without getting stuck
   * on crossbar or elevator
   */
  public final Distance minAlgaeInHeight = Meters.of(1.7);

  /**
   * How slow must the elevator move before it is considered to be stable at its goal position
   *
   * <p>This value is a unit because MetersPerSecond can't be serialized properly by JSONSync
   */
  public final Double maxElevatorSetpointVelocityMetersPerSecond = 0.01;

  /** The top of the height range where claw can hit crossbar */
  public final Distance crossbarTopCollisionHeight = Inches.of(-32.0);

  /** The bottom of the height range where claw can hit crossbar */
  public final Distance crossbarBottomCollisionHeight = Inches.of(-32.0);

  public static final class Sim {
    @JSONExclude
    public static final JSONSync<ElevatorConstants.Sim> synced =
        new JSONSync<ElevatorConstants.Sim>(
            new ElevatorConstants.Sim(),
            Filesystem.getDeployDirectory()
                .toPath()
                .resolve("constants/ElevatorConstants.Sim.json")
                .toString(),
            new JSONSyncConfigBuilder().build());

    /** Standard deviation passed to sim for the position measurement */
    public final Double positionStdDev = 0.0;

    /** Standard deviation passed to sim for the velocity measurement */
    public final Double velocityStdDev = 0.0;

    /**
     * Height the elevator sim is initialized to in meters, stored as a double so it can be synced
     * by JSONSync
     */
    public final Double elevatorStartingHeightMeters = 1.0;

    @JSONExclude
    public final Distance elevatorStartingHeight = Meters.of(elevatorStartingHeightMeters);
  }
}
