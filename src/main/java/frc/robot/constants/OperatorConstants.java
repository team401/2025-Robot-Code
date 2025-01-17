package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;
import edu.wpi.first.wpilibj.Filesystem;

public final class OperatorConstants {
    @JSONExclude
    public static final JSONSync<OperatorConstants> synced =
            new JSONSync<OperatorConstants>(
                    new OperatorConstants(),
                    Filesystem.getDeployDirectory()
                            .toPath()
                            .resolve("constants/OperatorConstants.json")
                            .toString(),
                    new JSONSync.JSONSyncConfigBuilder().build());

    public final Integer kDriverControllerPort = 0;
}
