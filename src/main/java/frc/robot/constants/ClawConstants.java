package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;

import com.ctre.phoenix6.signals.InvertedValue;
import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;
import coppercore.parameter_tools.JSONSyncConfigBuilder;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
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
          new JSONSyncConfigBuilder().build());

  public final Integer coralCANrangeID = 20; // TODO: Actual CAN id
  public final Integer algaeCANrangeID = 21; // TODO: Actual CAN id

  public final Integer clawMotorID = 14; // TODO: Finalize CAN id

  public final InvertedValue kClawMotorInverted =
      InvertedValue.Clockwise_Positive; // TODO: Tune this value

  public final Current clawSupplyCurrentLimit = Amps.of(60);
  public final Current clawStatorCurrentLimit = Amps.of(60);

  public final Distance coralProximityThreshold = Centimeters.of(5.0); // TODO: Tune these values!
  public final Distance coralProximityHysteresis = Centimeters.of(0.5);
  public final double coralMinSignalStrengthForValidMeasurement = 2500.0;

  public final Distance algaeProximityThreshold = Centimeters.of(5.0); // TODO: Tune these values!
  public final Distance algaeProximityHysteresis = Centimeters.of(0.5);
  public final double algaeMinSignalStrengthForValidMeasurement = 2500.0;
}
