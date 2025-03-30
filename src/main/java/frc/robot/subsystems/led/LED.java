package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/** LED subsystem for controlling addressable LEDs on the robot. */
public class LED extends SubsystemBase {

  private List<LEDPattern> leftPatterns = new ArrayList<>();
  private List<LEDPattern> rightPatterns = new ArrayList<>();

  private final AddressableLED led = new AddressableLED(LEDConstants.ledPort);
  public AddressableLEDBuffer ledStrip = new AddressableLEDBuffer(LEDConstants.totalLength);

  public AddressableLEDBufferView leftData =
      ledStrip
          .createView(0, LEDConstants.halfLength - 1)
          .reversed(); // these should both be reversed
  private AddressableLEDBufferView rightData =
      ledStrip.createView(LEDConstants.halfLength, LEDConstants.totalLength - 1).reversed();

  private Supplier<Boolean> visionWorkingSupplier = () -> true;

  private ScoringSubsystem scoringSubsystem;

  /**
   * Constructs the LED subsystem.
   *
   * @param scoringSubsystem The scoring subsystem
   * @param climbSubsystem The climb subsystem
   * @param drive The drive subsystem
   */
  public LED(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
    led.setLength(LEDConstants.totalLength);
    led.start();
  }

  /** Periodic method called every loop cycle. Updates LED patterns based on robot state. */
  @Override
  public void periodic() {
    if (!DriverStation.isDisabled()) {
      
      // Scoring Subsystem Checks
      if (scoringSubsystem != null) {
        if (scoringSubsystem.isCoralDetected() == true) {
          addPattern(LEDConstants.holdingAlgaePattern);
        } else if (scoringSubsystem.isAlgaeDetected() == true) {
          addPattern(LEDConstants.holdingCoralPattern);
        } else if (scoringSubsystem.isCoralDetected() == true
            && scoringSubsystem.isAlgaeDetected() == true) {
          addPattern(LEDConstants.holdingBothPattern);
        } else {
          addPattern(LEDConstants.clear);
        }
      }

      // check vision - bottom thirds
      if (!visionWorkingSupplier.get()) {
        addPattern(LEDConstants.isBeeLinkWorkingPattern);
      }

      if (DriverStation.getMatchTime() < 20 && DriverStation.getMatchTime() > 17) {
        addPattern(LEDConstants.endGame);
      }
      
      applyPatterns();

    } else {
      addPattern(LEDConstants.rainbowPattern);
      if (!visionWorkingSupplier.get()) {
        addPattern(LEDConstants.isBeeLinkWorkingPattern);
      }
      applyPatterns();
      LEDConstants.rainbowPattern.applyTo(ledStrip);
    }
    led.setData(ledStrip);
  }

  /**
   * Runs different LED patterns on each side of the LED strip.
   *
   * @param left Pattern for the left side
   * @param right Pattern for the right side
   */
  public void runSplitPattern(LEDPattern left, LEDPattern right) {
    right.applyTo(leftData);
    left.applyTo(rightData);
  }

  /**
   * Runs the same LED pattern on both sides of the LED strip.
   *
   * @param pattern The LED pattern to display
   */
  public void runPattern(LEDPattern pattern) {
    runSplitPattern(pattern, pattern);
  }

  public void setVisionWorkingSupplier(Supplier<Boolean> visionWorkingSupplier) {
    this.visionWorkingSupplier = visionWorkingSupplier;
  }

  /**
   * Adds separate patterns for the left and right LED strips.
   *
   * <p>Added to final pattern in {@link #applyPatterns()}
   *
   * @param left Pattern for the left LEDs
   * @param right Pattern for the right LEDs
   */
  public void addSplitPattern(LEDPattern left, LEDPattern right) {
    leftPatterns.add(left);
    rightPatterns.add(right);
  }

  /**
   * Adds the same pattern to both left and right LED strips.
   *
   * <p>Added to final pattern in {@link #applyPatterns()}
   *
   * @param pattern The LED pattern to add
   */
  public void addPattern(LEDPattern pattern) {
    leftPatterns.add(pattern);
    rightPatterns.add(pattern);
  }

  /** Applies all active LED patterns by overlaying them and displaying the final effect. */
  public void applyPatterns() {

    LEDPattern leftFinal = LEDPattern.solid(Color.kBlack);
    LEDPattern rightFinal = LEDPattern.solid(Color.kBlack);

    for (LEDPattern pattern : leftPatterns) {
      leftFinal = pattern.overlayOn(leftFinal);
    }
    for (LEDPattern pattern : rightPatterns) {
      rightFinal = pattern.overlayOn(rightFinal);
    }

    leftFinal.applyTo(leftData);
    rightFinal.applyTo(rightData);

    leftPatterns.clear();
    rightPatterns.clear();
  }

  public void setLedOn(boolean on) {
    if (on) {
      led.start();
    } else {
      led.stop();
    }
  }
}
