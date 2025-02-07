package frc.robot.constants.autoStrategies;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.AutoStrategy;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import java.util.LinkedList;
import java.util.List;

public class ExampleAutoPath implements AutoStrategy {
  @JSONExclude public static final String autoStrategyName = "ExampleAutoPath";
  public static final JSONSync<ExampleAutoPath> synced =
      new JSONSync<ExampleAutoPath>(
          new ExampleAutoPath(),
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("constants/autoStrategies" + autoStrategyName + "/.json")
              .toString(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final List<DesiredLocation> scoringLocations = new LinkedList<>();
  public final List<Integer> scoringLevels = new LinkedList<>();
  public final DesiredLocation intakeLocation = DesiredLocation.CoralStationLeft;
}
