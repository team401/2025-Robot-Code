package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class WristConstants {
  @JSONExclude
  public static final JSONSync<WristConstants> synced =
      new JSONSync<WristConstants>(
          new WristConstants(),
          "WristConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().build());

  public final Integer wristMotorId = 13;
  public final Integer wristCANcoderId = 22;

  /**
   * FusedCANcoder sensor to mechanism ratio
   *
   * <p>see talonFX docs:
   * https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/FeedbackConfigs.html#SensorToMechanismRatio
   */
  public final Double sensorToMechanismRatio = 1.0; // TODO: Actual value

  public final Double wristReduction = 20.0;

  @JSONExclude public final Double rotorToSensorRatio = wristReduction;

  public final InvertedValue wristMotorInvertedValue = InvertedValue.Clockwise_Positive;

  public final NeutralModeValue wristNeutralModeValue = NeutralModeValue.Brake;

  public final Current wristSupplyCurrentLimit = Amps.of(40.0);
  public final Current wristStatorCurrentLimit = Amps.of(40.0);

  /** Peak forward and reverse current for FieldOriented */
  public final Current peakFOCCurrent = Amps.of(40.0);

  public final Double wristKG = 0.0; // TODO: Automatic feedforward characterization
  public final Double wristKS = 0.0;
  public final Double wristKV = 0.0;
  public final Double wristKA = 0.0;

  public final Double wristKP = 0.0; // TODO: Tune these
  public final Double wristKI = 0.0;
  public final Double wristKD = 0.0;

  // This value is a a Double because RotationsPerSecond doesn't serialize properly with JSONSync
  public final Double wristMotionMagicCruiseVelocityRotationsPerSecond = 0.1;

  // TODO: Tune this!

  public final Double wristMotionMagicExpo_kA = 6.0; //
  public final Double wristMotionMagicExpo_kV = 6.0; //

  public final Angle wristCANcoderAbsoluteSensorDiscontinuityPoint =
      Rotations.of(0.3); // TODO: Confirm this
  public final Angle wristCANcoderMagnetOffset = Rotations.of(0.0); // TODO: Tune this value
  public final SensorDirectionValue wristCANcoderSensorDirection =
      SensorDirectionValue.CounterClockwise_Positive;

  // These clamps are the default clamps for the wrist, as well as limiting the moving clamps of the
  // wrist themselves.
  public final Angle wristMinMinAngle = Rotations.of(-0.8);
  public final Angle wristMaxMaxAngle = Rotations.of(0.25);

  /** The maximum angle the wrist can be at while passing by the crossbar without hitting it. */
  public final Angle maxCrossBarSafeAngle = Rotations.of(0.0); // TODO: Actual value

  /**
   * The minimum angle the wrist can be at while the elevator is down and not hit the parts of the
   * robot below.
   */
  public final Angle minElevatorDownSafeAngle = Rotations.of(-0.5); // TODO: Actual value

  /** The wrist can be this far away from the goal and considered "at the setpoint" */
  public final Angle wristSetpointEpsilon = Degrees.of(5.0);

  public static final class Sim {
    @JSONExclude
    public static final JSONSync<WristConstants.Sim> synced =
        new JSONSync<WristConstants.Sim>(
            new WristConstants.Sim(),
            Filesystem.getDeployDirectory() // Don't use environment handler for sim constants
                .toPath()
                .resolve("constants/WristConstants.Sim.json")
                .toString(),
            new JSONSyncConfigBuilder().build());

    // This value is a Double because MomentOfInertia units don't serialize properly with JSONSync
    public final Double wristMomentOfInertiaKgM2 = 0.0672304487;

    @JSONExclude
    public final MomentOfInertia wristMomentOfInertia =
        KilogramSquareMeters.of(wristMomentOfInertiaKgM2);

    public final Distance wristArmLength = Meters.of(0.5);

    public final Angle wristMinAngle = Rotations.of(-0.80);
    public final Angle wristMaxAngle = Rotations.of(0.30);

    public final Angle wristStartingAngle = Rotations.of(0.0);
  }
}
