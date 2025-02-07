package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import java.util.LinkedList;
import java.util.List;

public class AutoPath {
  public static final JSONSync<AutoPath> synced =
      new JSONSync<AutoPath>(
          new AutoPath(),
          Filesystem.getDeployDirectory().toPath().resolve("constants/AutoPath.json").toString(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final List<DesiredLocation> scoringLocations = new LinkedList<>();
  public final DesiredLocation intakeLocation = DesiredLocation.CoralStationLeft;
}
