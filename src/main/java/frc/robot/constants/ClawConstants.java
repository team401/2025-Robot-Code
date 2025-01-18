package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;
import edu.wpi.first.wpilibj.Filesystem;

public class ClawConstants {
    @JSONExclude
    public static final JSONSync<ClawConstants> synced =
            new JSONSync<ClawConstants>(
                    new ClawConstants(),
                    Filesystem.getDeployDirectory()
                            .toPath()
                            .resolve("constants/ClawConstants.json")
                            .toString(),
                    new JSONSync.JSONSyncConfigBuilder().build());

    public final Integer coralCANrangeID = 20; // TODO: Actual CAN id
    public final Integer algaeCANrangeID = 21; // TODO: Actual CAN id

    public final Integer clawMotorID = 14; // TODO: Finalize CAN id
}
