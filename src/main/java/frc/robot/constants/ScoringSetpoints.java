package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import coppercore.math.InterpolateDouble;
import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import java.util.HashMap;

public class ScoringSetpoints {
  /** Keeps track of a setpoint for the scoring subsystem. */
  public static record ScoringSetpoint(String name, Distance elevatorHeight, Angle wristAngle) {}

  @JSONExclude
  public static final JSONSync<ScoringSetpoints> synced =
      new JSONSync<ScoringSetpoints>(
          new ScoringSetpoints(),
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("constants/ScoringSetpoints.json")
              .toString(),
          new JSONSyncConfigBuilder().build());

  public final ScoringSetpoint idle =
      new ScoringSetpoint(
          "Idle",
          Inches.of(1.5),
          Rotations.of(0.3214)); // Idle at 1.5 inches to avoid obstructing cameras

  public final ScoringSetpoint idleWithAlgae =
      new ScoringSetpoint(
          "IdleWithAlgae",
          Meters.of(0.3),
          Rotations.of(-0.105)); // Hold algae with the wrist out and the arm up

  public final ScoringSetpoint L1 = new ScoringSetpoint("L1", Meters.of(0.15), Rotations.of(0.3));
  public final ScoringSetpoint L2 = new ScoringSetpoint("L2", Meters.of(0.29), Rotations.of(0.29));
  public final ScoringSetpoint L3 = new ScoringSetpoint("L3", Meters.of(0.7), Rotations.of(0.29));
  public final ScoringSetpoint L4 = new ScoringSetpoint("L4", Meters.of(1.6), Rotations.of(0.09));

  public final ScoringSetpoint farWarmup =
      new ScoringSetpoint("farWarmup", Meters.of(0.65), Rotations.of(0.3));

  // TODO: Algae setpoints
  public final ScoringSetpoint L2algae =
      new ScoringSetpoint("L2algae", Meters.of(0.55), Rotations.of(-0.095));
  public final ScoringSetpoint L3algae =
      new ScoringSetpoint("L3algae", Meters.of(0.97393), Rotations.of(-0.095));

  public final ScoringSetpoint coralStation =
      new ScoringSetpoint("coralStation", Meters.of(0.0), Rotations.of(0.3214));

  public final ScoringSetpoint processor =
      new ScoringSetpoint("processor", Meters.of(0.18), Rotations.of(-0.15));

  public final ScoringSetpoint netWarmup =
      new ScoringSetpoint("netWarmup", Meters.of(1.8), Radians.of(1.3));
  public final ScoringSetpoint net = new ScoringSetpoint("net", Meters.of(1.9), Rotations.of(0.0));

  public final ScoringSetpoint ground =
      new ScoringSetpoint("ground", Meters.of(0.2), Rotations.of(0.0));

  /**
   * Combined with variableL4WristAngles, these form the interpolate double used for variable
   * distance L4 scoring
   */
  public final Distance[] variableL4AlongTrackDistances = {Meters.of(0.1), Meters.of(0.0)};

  /**
   * Combined with variableL4AlongTrackDistances, these form the interpolate double used for
   * variable distance L4 scoring
   */
  public final Angle[] variableL4WristAngles = {Rotations.of(0.32), Rotations.of(0.09)};

  /** Maps along track distances (in meters) to wrist angles (in rotations) */
  public static InterpolateDouble variableL4WristMap;

  /** Keep track of a reference to the drive subsystem to access along track distance */
  @JSONExclude private Drive driveReference = null;

  /**
   * Keep track of whether or not we are currently using variable L4 setpoints. This can be false if
   * a malformed setpoint map causes variable L4 setpoints to be disabled.
   */
  @JSONExclude private boolean usingVariableL4 = true;

  /** Store maximum distance included in the variable L4 map to see if drive is within range */
  @JSONExclude private double maxVariableDistance = 0.0;

  /**
   * Give the ScoringSetpoints instance a reference to the drivetrain, used to obtain along track
   * distance
   *
   * @param driveReference The Drive subsystem instance
   */
  public void setDriveReference(Drive driveReference) {
    this.driveReference = driveReference;
  }

  /**
   * Create the InterpolateDouble to map along track distances to wrist angle.
   *
   * <p>This must be called AFTER syncing the constants with JSON. This method fills the loaded
   * constants on an INSTANCE of ScoringSetpoints. Make sure it is called on an instance that has
   * synced its JSON constants already.
   *
   * <p>This also fills in the maximum distance to be used by drive
   *
   * @throws Exception If constants are incorrectly defined with different lengths, an exception
   *     will be thrown
   */
  public void fillVariableL4WristMap() throws Exception {
    HashMap<Double, Double> wristMap = new HashMap<>();

    if (variableL4AlongTrackDistances.length != variableL4WristAngles.length) {
      throw new Exception(
          "Variable L4 Along Track distances and Wrist angles have different lengths. Must be a 1:1 mapping.");
    }

    for (int i = 0; i < variableL4AlongTrackDistances.length; i++) {
      double distanceMeters = variableL4AlongTrackDistances[i].in(Meters);
      wristMap.put(distanceMeters, variableL4WristAngles[i].in(Rotations));

      if (distanceMeters > maxVariableDistance) {
        maxVariableDistance = distanceMeters;
      }
    }

    variableL4WristMap = new InterpolateDouble(wristMap);

    usingVariableL4 = true;
  }

  /**
   * Disable the use of variable L4 setpoints for this ScoringSetpoints instance.
   *
   * <p>This should be called if there is a failure to create the setpoint map.
   */
  public void disableVariableL4() {
    this.usingVariableL4 = false;
  }

  public static ScoringSetpoint getVariableL4Setpoint(double alongTrackDistance) {
    Angle wristAngle = Rotations.of(variableL4WristMap.getValue(alongTrackDistance));

    return new ScoringSetpoint(
        "L4Variable", JsonConstants.scoringSetpoints.L4.elevatorHeight(), wristAngle);
  }

  /**
   * Given a fieldTarget, return the correct warmup setpoint
   *
   * @return
   */
  public ScoringSetpoint getWarmupSetpoint(FieldTarget target) {
    switch (target) {
      case L1:
        return JsonConstants.scoringSetpoints.L1;
      case L2:
        return JsonConstants.scoringSetpoints.L2;
      case L3:
        return JsonConstants.scoringSetpoints.L3;
      case L4:
        if (usingVariableL4()) {
          double alongTrackDistance = 0.0;

          if (driveReference != null) {
            alongTrackDistance = driveReference.getLatestAlongTrackDistance();
          }

          return getVariableL4Setpoint(alongTrackDistance);
        } else {
          return JsonConstants.scoringSetpoints.L4;
        }
      case Net:
        return JsonConstants.scoringSetpoints.netWarmup;
      case Processor:
        return JsonConstants.scoringSetpoints.processor;
      case Ground:
      default:
        System.out.println(
            "ERROR: Can't warmup for FieldTarget " + target + ", defaulting to idle");
        return JsonConstants.scoringSetpoints.idle;
    }
  }

  /**
   * Is the robot currently using variable L4 setpoints
   *
   * @return True if featureflags AND scoring setpoint validation BOTH say that L4 setpoints are
   *     used
   */
  public boolean usingVariableL4() {
    return JsonConstants.scoringFeatureFlags.useVariableL4Setpoint && usingVariableL4;
  }

  /**
   * Get the furthest distance there is a defined setpoint for in the variable L4 map
   *
   * @return The largest distance in the variable L4 map
   */
  public double getMaxVariableDistance() {
    return maxVariableDistance;
  }
}
