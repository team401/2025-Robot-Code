package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;

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
          Meters.of(0.2),
          Rotations.of(-0.75)); // Hold algae with the wrist out and the arm up

  public final ScoringSetpoint L1 = new ScoringSetpoint("L1", Meters.of(1.0), Rotations.of(0.3214));
  public final ScoringSetpoint L2 = new ScoringSetpoint("L2", Meters.of(1.5), Rotations.of(0.3214));
  public final ScoringSetpoint L3 = new ScoringSetpoint("L3", Meters.of(2.0), Rotations.of(0.3214));
  public final ScoringSetpoint L4 = new ScoringSetpoint("L4", Meters.of(2.5), Rotations.of(0.3214));

  public final ScoringSetpoint farWarmup =
      new ScoringSetpoint("farWarmup", Meters.of(0.6), Rotations.of(0.3214));

  // TODO: Algae setpoints
  public final ScoringSetpoint L2algae =
      new ScoringSetpoint("L2algae", Meters.of(1.8), Rotations.of(-0.75));
  public final ScoringSetpoint L3algae =
      new ScoringSetpoint("L3algae", Meters.of(2.3), Rotations.of(-0.75));

  public final ScoringSetpoint coralStation =
      new ScoringSetpoint("coralStation", Meters.of(0.1), Rotations.of(0.25));

  public final ScoringSetpoint processor =
      new ScoringSetpoint("processor", Meters.of(0.1), Rotations.of(-0.75));

  public final ScoringSetpoint netWarmup =
      new ScoringSetpoint("netWarmup", Meters.of(0.25), Rotations.of(-0.105));
  public final ScoringSetpoint net =
      new ScoringSetpoint("net", Meters.of(1.25), Rotations.of(-0.75));

  public final ScoringSetpoint ground =
      new ScoringSetpoint("ground", Meters.of(0.2), Rotations.of(-0.75));

  /**
   * Given a fieldTarget, return the correct warmup setpoint
   *
   * @return
   */
  public static ScoringSetpoint getWarmupSetpoint(FieldTarget target) {
    switch (target) {
      case L1:
        return JsonConstants.scoringSetpoints.L1;
      case L2:
        return JsonConstants.scoringSetpoints.L2;
      case L3:
        return JsonConstants.scoringSetpoints.L3;
      case L4:
        return JsonConstants.scoringSetpoints.L4;
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
}
