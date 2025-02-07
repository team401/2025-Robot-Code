import java.util.LinkedList;
import java.util.List;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.Drive.DesiredLocation;

public class AutoPath {
    public static final JSONSync<AutoPath> synced =
    new JSONSync<AutoPath>(
        new AutoPath(),
        Filesystem.getDeployDirectory()
            .toPath()
            .resolve("constants/BlueFieldLocations.json")
            .toString(),
        new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

    public final List<DesiredLocation> scoringLocations = new LinkedList<>();
}
