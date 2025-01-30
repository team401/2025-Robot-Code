package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;

public class ScoringSetpoints {
  /** Keeps track of a setpoint for the scoring subsystem. */
  public static record ScoringSetpoint(Distance elevatorHeight) {}

  // TODO: Add wrist angle to ScoringSetpoint

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
      new ScoringSetpoint(Inches.of(1.5)); // Idle at 1.5 inches to avoid obstructing cameras

  // TODO: Get actual values for all values from here down
  public final ScoringSetpoint L1 = new ScoringSetpoint(Meters.of(0.05));
  public final ScoringSetpoint L2 = new ScoringSetpoint(Meters.of(0.25));
  public final ScoringSetpoint L3 = new ScoringSetpoint(Meters.of(0.5));
  public final ScoringSetpoint L4 = new ScoringSetpoint(Meters.of(0.75));

  public final ScoringSetpoint L2algae = new ScoringSetpoint(Meters.of(0.2));
  public final ScoringSetpoint L3algae = new ScoringSetpoint(Meters.of(0.4));

  public final ScoringSetpoint coralStation = new ScoringSetpoint(Meters.of(1.0));

  public final ScoringSetpoint processor = new ScoringSetpoint(Meters.of(0.1));
  public final ScoringSetpoint net = new ScoringSetpoint(Meters.of(1.25));

  public final ScoringSetpoint ground = new ScoringSetpoint(Meters.of(0.0));
}
