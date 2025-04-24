package frc.robot.constants; // NOTE: This should be changed if you keep your constants in

// a separate package from your code

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.Filesystem;

public final class WristConstants {
  @JSONExclude
  public static final JSONSync<WristConstants> synced =
      new JSONSync<WristConstants>(
          new WristConstants(),
          "WristConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Integer wristMotorId = 13;

  /**
   * What point in the sensor's range the discontinuity occurs. Results in a range of [1-x, x). For
   * example, a value of 1 gives a range of [0.0, 1).
   */
  public final Double wristEncoderDiscontinuityPoint = 1.0;

  public final Angle wristEncoderMagnetOffset = Radians.of(0.0);

  public final Integer wristEncoderID = 22;

  public final SensorDirectionValue wristEncoderDirection = SensorDirectionValue.Clockwise_Positive;

  public final Angle wristMinMinAngle = Radians.of(-1.68);
  public final Angle wristMaxMaxAngle = Radians.of(2.0);

  /*
   * The wristEncoder is represented as the mechanism in our Phoenix configs.
   * This means that we are controlling to a goal in terms of large CANCoder angle.
   */
  @JSONExclude public final double wristEncoderToMechanismRatio = 1.0;

  @JSONExclude
  public final double rotorToWristEncoderRatio = 20.0; // TODO: Replace placeholder value

  public final Double wristkP = 25.0;
  public final Double wristkI = 1.0;
  public final Double wristkD = 0.5;

  public final Double wristkS = 0.34;
  public final Double wristkV = 1.0;
  public final Double wristkA = 0.0;
  public final Double wristkG = 0.34;

  /** This is a Double until coppercore JSONSync supports RotationsPerSecond */
  public final Double wristAngularCruiseVelocityRotationsPerSecond = 3.0;

  /*
   * The Motion Magic Expo kV, measured in Volts per Radian per Second, but represented as a double so it can be synced by JSONSync
   *
   * <p> This kV is used by Motion Magic Expo to generate a motion profile. Dividing the supply voltage by
   * kV results in the maximum velocity of the system. Therefore, a higher profile kV results in a
   * lower profile velocity.
   */
  public final Double wristExpo_kV_raw = 0.1;

  /** The angle at which the wrist will collide with the crossbar when rotating downward */
  public final Angle crossbarTopCollisionAngle = Rotations.of(0.32);

  /** The angle at which the wrist will collide with the crossbar when rotating upward */
  public final Angle crossbarBottomCollisionAngle = Rotations.of(0.32);

  public final Angle algaeUnderCrossbarAngle = Radians.of(-0.105);
  public final Angle minElevatorDownSafeAngle = Radians.of(-0.503);
  public final Distance closeToReefThreshold = Meters.of(2.5);
  public final Angle minReefSafeAngle = Rotations.of(0.3);
  public final Angle netShotRollerWristEpsilon = Rotations.of(0.05);
  public final Double maxWristSetpointVelocityRotationsPerSecond = 0.008333;
  public final Double wristStableDebounceTimeSeconds = 0.5;
  public final Angle wristSetpointEpsilon = Degrees.of(5.0);
  /*
   * The Motion Magic Expo kA, measured in Volts per Radian per Second Squared, but represented as a double so it can be synced by JSONSync
   */
  public final Double wristExpo_kA_raw = 0.4;

  public final Current wristStatorCurrentLimit =
      Amps.of(40.0); // TODO: Replace placeholder current limit

  public final Double wristReduction = 20.0; // TODO: Replace placeholder reduction

  public static final class Sim {
    @JSONExclude
    public static final JSONSync<WristConstants.Sim> synced =
        new JSONSync<WristConstants.Sim>(
            new WristConstants.Sim(),
            Filesystem.getDeployDirectory()
                .toPath()
                .resolve("constants/WristConstants.Sim.json")
                .toString(),
            new JSONSyncConfigBuilder().build());

    /** Standard deviation passed to sim for the position measurement */
    public final Double positionStdDev = 0.0;

    /** Standard deviation passed to sim for the velocity measurement */
    public final Double velocityStdDev = 0.0;

    public final Double wristMomentOfInertiaKgM2 = 0.0672304487;

    @JSONExclude
    public final MomentOfInertia wristMomentOfInertia =
        KilogramSquareMeters.of(
            wristMomentOfInertiaKgM2); // TODO: Replace placeholder moment of inertia

    public final Distance wristArmLength = Meters.of(0.5); // TODO: Replace placeholder arm length
    public final Angle wristMinAngle =
        Radians.of(-1.68); // TODO: Update placeholder min & max angles
    public final Angle wristMaxAngle = Radians.of(2.0);

    public final Angle wristStartingAngle = Radians.of(0.0);
  }
}
