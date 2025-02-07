package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import frc.robot.constants.autoStrategies.ExampleAutoPath;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import java.util.List;

public interface AutoPath {
  public static final String autoPathName = "";
  public static final JSONSync<ExampleAutoPath> synced = null;

  public final List<DesiredLocation> scoringLocations = null;
  public final DesiredLocation intakeLocation = null;
}
