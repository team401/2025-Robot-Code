package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import frc.robot.constants.autoStrategies.ExampleAutoPath;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import java.util.List;
import java.util.Queue;

public interface AutoPath {
  public enum ActionType {
    INTAKE,
    SCORE
  };
  public static final String autoPathName = "";
  public static final JSONSync<ExampleAutoPath> synced = null;

  public final List<DesiredLocation> scoringLocations = null;
  public final DesiredLocation intakeLocation = null;

  public static record Action (ActionType type, DesiredLocation location) {}
}
