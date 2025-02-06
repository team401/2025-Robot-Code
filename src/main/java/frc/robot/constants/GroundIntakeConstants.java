package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;

public class GroundIntakeConstants {
  @JSONExclude
  public static final JSONSync<GroundIntakeConstants> synced =
      new JSONSync<GroundIntakeConstants>(
          new GroundIntakeConstants(),
          "GroundIntakeConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().build());

  public final Integer shoulderMotorID = 14; // TODO: Finalize CAN id
  public final Integer rollerMotorID = 15; // TODO: Finalize CAN id
  
  public final InvertedValue kRollerMotorInverted =
  InvertedValue.Clockwise_Positive; // TODO: Tune this value

  public final InvertedValue kShoulderMotorInverted =
      InvertedValue.Clockwise_Positive; // TODO: Tune this value

  public final Current shoulderMotorSupplyCurrentLimit = Amps.of(60);
  public final Current shoulderMotorStatorCurrentLimit = Amps.of(60);

  public final Current rollerMotorSupplyCurrentLimit = Amps.of(60);
  public final Current rollerMotorStatorCurrentLimit = Amps.of(60);

  public final Voltage rollerCollectVoltage = Volts.of(3.0);
  public final Voltage rollerSpitVoltage = Volts.of(-3.0);

  

  public static final class Sim {
    @JSONExclude
    public static final JSONSync<GroundIntakeConstants.Sim> synced =
        new JSONSync<GroundIntakeConstants.Sim>(
            new GroundIntakeConstants.Sim(),
            Filesystem.getDeployDirectory()
                .toPath()
                .resolve("constants/GroundIntakeConstants.Sim.json")
                .toString(),
            new JSONSyncConfigBuilder().build());

    /** How long the motor must remain powered to intake or outtake a game piece */
    public final Double actionTimeSeconds = 1.0;

    /**
     * How many rotations per second the motor will spin per volt applied. This isn't a unit because
     * I couldn't fight the double-layered PerUnit mess to get it to do what I want.
     */
    public final double rotationsPerSecondPerVolt = 1.0;

    /** When piecePos > coralDetectionPoint, the simulated sensor can sense it */
    public final double coralDetectionPoint = 0.7;

    /** When piecePos < algaeDetectionPoint, the simulated sensor can sense it */
    public final Double algaeDetectionPoint = 0.7;
  }
}
