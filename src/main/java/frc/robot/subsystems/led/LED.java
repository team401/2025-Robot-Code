package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

public class LED extends SubsystemBase {

  // Enum for different LED modes.
  public enum LEDMode {
    DEFAULT,
    RAINBOW,
    LOCKED_ON_HANG,
    HOLDING_ALGAE,
    HOLDING_CORAL,
    TARGET_ON_REEF,
    TARGET_ON_REEF_L1,
    TARGET_ON_REEF_L2,
    TARGET_ON_REEF_L3,
    TARGET_ON_REEF_L4,
    TARGET_ON_PROCESSOR,
    TARGET_ON_NET,
    TARGET_ON_CORAL
  }

  private final AddressableLED ledPort = new AddressableLED(LEDConstants.ledPort);
  private final AddressableLEDBuffer ledStrip = new AddressableLEDBuffer(LEDConstants.totalLength);

  // Create views for two sections of the LED strip.
  private final AddressableLEDBufferView bottomData = ledStrip.createView(0, 29);
  // Create the top view starting at LED 253; then reverse the view if needed.
  private final AddressableLEDBufferView topData = ledStrip.createView(30, 60).reversed();

  // Define patterns for different effects.
  public LEDPattern rainbow =
      LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(50));

  public LEDPattern lockedOnHang = LEDPattern.solid(LEDConstants.lockedOnHang);

  public LEDPattern holdingAlgae =
      LEDPattern.steps(
          Map.of(
              0, LEDConstants.holdingAlgae, 1 / 3.0, LEDConstants.off));

  public LEDPattern holdingCoral =
      LEDPattern.steps(
          Map.of(
              0, LEDConstants.holdingCoral, 1 / 3.0, LEDConstants.off, 2 / 3.0, LEDConstants.off));

  public LEDPattern targetOnReef =
      LEDPattern.steps(
          Map.of(
              0,
              LEDConstants.off,
              1 / 3.0,
              LEDConstants.targetOnReef,
              2 / 3.0,
              LEDConstants.targetOnReef));

  public LEDPattern targetOnReefL1 =
      LEDPattern.steps(
          Map.of(
              0,
              LEDConstants.off,
              1 / 3.0,
              LEDConstants.targetOnReefL1,
              2 / 3.0,
              LEDConstants.targetOnReefL1));

  public LEDPattern targetOnReefL2 =
      LEDPattern.steps(
          Map.of(
              0,
              LEDConstants.off,
              1 / 3.0,
              LEDConstants.targetOnReefL2,
              2 / 3.0,
              LEDConstants.targetOnReefL2));

  public LEDPattern targetOnReefL3 =
      LEDPattern.steps(
          Map.of(
              0,
              LEDConstants.off,
              1 / 3.0,
              LEDConstants.targetOnReefL3,
              2 / 3.0,
              LEDConstants.targetOnReefL3));

  public LEDPattern targetOnReefL4 =
      LEDPattern.steps(
          Map.of(
              0,
              LEDConstants.off,
              1 / 3.0,
              LEDConstants.targetOnReefL4,
              2 / 3.0,
              LEDConstants.targetOnReefL4));

  public LEDPattern targetOnProcessor =
      LEDPattern.steps(
          Map.of(
              0,
              LEDConstants.off,
              1 / 3.0,
              LEDConstants.targetOnProcessor,
              2 / 3.0,
              LEDConstants.targetOnProcessor));

  public LEDPattern targetOnNet =
      LEDPattern.steps(
          Map.of(
              0,
              LEDConstants.off,
              1 / 3.0,
              LEDConstants.targetOnNet,
              2 / 3.0,
              LEDConstants.targetOnNet));

  public LEDPattern targetOnCoral =
      LEDPattern.steps(
          Map.of(
              0,
              LEDConstants.off,
              1 / 3.0,
              LEDConstants.targetOnCoral,
              2 / 3.0,
              LEDConstants.targetOnCoral));

  // Map LEDMode enums to their respective LEDPattern.
  private final Map<LEDMode, LEDPattern> ledPatterns =
      Map.ofEntries(
          Map.entry(LEDMode.RAINBOW, rainbow),
          Map.entry(LEDMode.LOCKED_ON_HANG, lockedOnHang),
          Map.entry(LEDMode.HOLDING_ALGAE, holdingAlgae),
          Map.entry(LEDMode.HOLDING_CORAL, holdingCoral),
          Map.entry(LEDMode.TARGET_ON_REEF, targetOnReef),
          Map.entry(LEDMode.TARGET_ON_REEF_L1, targetOnReefL1),
          Map.entry(LEDMode.TARGET_ON_REEF_L2, targetOnReefL2),
          Map.entry(LEDMode.TARGET_ON_REEF_L3, targetOnReefL3),
          Map.entry(LEDMode.TARGET_ON_REEF_L4, targetOnReefL4),
          Map.entry(LEDMode.TARGET_ON_PROCESSOR, targetOnProcessor),
          Map.entry(LEDMode.TARGET_ON_NET, targetOnNet),
          Map.entry(LEDMode.TARGET_ON_CORAL, targetOnCoral));

  public LED() {
    ledPort.setLength(LEDConstants.totalLength);
    ledPort.start();
  }

  @Override
  public void periodic() {
    ledPort.setData(ledStrip);
  }

  /** Runs a single pattern on both sides. */
  public Command runPattern(LEDPattern pattern) {
    return runCustomSplitPattern(pattern, pattern);
  }

  /** Runs separate patterns on the top and bottom LED sections. */
  public Command runCustomSplitPattern(LEDPattern up, LEDPattern down) {
    return run(
        () -> {
          down.applyTo(bottomData);
          up.applyTo(topData);
        });
  }

  /** Runs LED patterns based on the LEDMode enum for each side. */
  public Command runModeBasedSplitPattern(LEDMode upMode, LEDMode downMode) {
    return run(
        () -> {
          LEDPattern upPattern = ledPatterns.get(upMode);
          LEDPattern downPattern = ledPatterns.get(downMode);
          downPattern.applyTo(bottomData);
          upPattern.applyTo(topData);
        });
  }

  public Command runCycle() {
    return new Command() {
      private double lastTime;
      private int currentIndex = 0;

      // Create a list of solid LEDPatterns using your LEDConstants.
      // Add or remove colors as desired.
      private final List<LEDPattern> patterns =
          Arrays.asList(
              LEDPattern.solid(LEDConstants.lockedOnHang),
              LEDPattern.solid(LEDConstants.holdingAlgae),
              LEDPattern.solid(LEDConstants.holdingCoral),
              LEDPattern.solid(LEDConstants.targetOnReefL1),
              LEDPattern.solid(LEDConstants.targetOnReefL2),
              LEDPattern.solid(LEDConstants.targetOnReefL3),
              LEDPattern.solid(LEDConstants.targetOnReefL4),
              LEDPattern.solid(LEDConstants.targetOnReef),
              LEDPattern.solid(LEDConstants.targetOnNet));

      @Override
      public void initialize() {
        currentIndex = 0;
        lastTime = Timer.getFPGATimestamp();
        // Apply the first color pattern on both sides.
        runCustomSplitPattern(patterns.get(currentIndex), patterns.get(currentIndex)).schedule();
      }

      @Override
      public void execute() {
        double now = Timer.getFPGATimestamp();
        if (now - lastTime >= 5.5) { // One second elapsed
          currentIndex = (currentIndex + 1) % patterns.size();
          if (currentIndex < patterns.size()) {
            // Schedule the next color.
            runCustomSplitPattern(
                    patterns.get(currentIndex), patterns.get((currentIndex + 1) % patterns.size()))
                .schedule();
            lastTime = now;
          }
        }
      }

      @Override
      public boolean isFinished() {
        return false;
      }

      @Override
      public void end(boolean interrupted) {
        // Optionally, cancel any running LED command if needed.
      }
    };
  }
}
