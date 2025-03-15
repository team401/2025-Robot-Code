package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Filesystem;

public class ClawConstants {
  @JSONExclude
  public static final JSONSync<ClawConstants> synced =
      new JSONSync<ClawConstants>(
          new ClawConstants(),
          "ClawConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().build());

  public final Integer coralCANrangeID = 20;
  public final Integer algaeCANrangeID = 21;

  public final Integer clawMotorID = 14;

  public final InvertedValue kClawMotorInverted = InvertedValue.Clockwise_Positive;

  public final Current clawSupplyCurrentLimit = Amps.of(60);
  public final Current clawStatorCurrentLimit = Amps.of(60);

  public final Distance coralProximityThreshold = Centimeters.of(8.0);
  public final Distance coralProximityHysteresis = Centimeters.of(0.5);
  public final double coralMinSignalStrengthForValidMeasurement = 2500.0;

  public final Distance algaeProximityThreshold = Centimeters.of(5.0); // TODO: Tune this value
  public final Distance algaeProximityHysteresis = Centimeters.of(0.5);
  public final double algaeMinSignalStrengthForValidMeasurement = 2500.0;

  public final Voltage coralIntakeVoltage = Volts.of(3.0);
  public final Voltage algaeIntakeVoltage = Volts.of(-3.0);
  public final Voltage coralScoreVoltage = Volts.of(3.0);
  public final Voltage coralL23ScoreVoltage = Volts.of(6.0);
  public final Voltage algaeScoreVoltage = Volts.of(-3.0);

  /** Voltage the claw should use to hold the algae in place while idling */
  public final Voltage algaeIdleVoltage = Volts.of(1.0);

  /** How far should the motor rotate after the sensor detects a coral to intake? */
  public final Angle intakeAnglePastCoralrange = Rotations.of(3.375); // 2.275 on 3 volts

  /** How far should the motor rotate after the sensor detects an algae to intake? */
  public final Angle intakeAnglePastAlgaerange = Rotations.of(2.0);

  public final Current algaeDetectionCurrent = Amps.of(10.0);

  public final Time algaeCurrentDetectionTimeRising = Seconds.of(0.5);

  public final Time algaeCurrentDetectionTimeFalling = Seconds.of(2.0);

  public static final class Sim {
    @JSONExclude
    public static final JSONSync<ClawConstants.Sim> synced =
        new JSONSync<ClawConstants.Sim>(
            new ClawConstants.Sim(),
            Filesystem.getDeployDirectory()
                .toPath()
                .resolve("constants/ClawConstants.Sim.json")
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
