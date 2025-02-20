package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.ModeConstants.Mode;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

public class LED extends SubsystemBase {

  private boolean enabled = true;
  private ScoringSubsystem scoringSubsystem;
  private ClimbSubsystem climbSubsystem;

  private final AddressableLED led = new AddressableLED(LEDConstants.ledPort);
  private final AddressableLEDBuffer ledStrip = new AddressableLEDBuffer(LEDConstants.totalLength);

  private final AddressableLEDBufferView leftData = ledStrip.createView(0, 30);
  private final AddressableLEDBufferView rightData = ledStrip.createView(30, 60).reversed();

  private List<LEDPattern> leftPatterns = new ArrayList<>();
  private List<LEDPattern> rightPatterns = new ArrayList<>();

  public LEDPattern rainbow =
      LEDPattern.rainbow(255, 255)
          .scrollAtRelativeSpeed(Percent.per(Second).of(LEDConstants.rainbowSpeed));
  public LEDPattern clear = LEDPattern.solid(Color.kBlack);

  public LEDPattern holdingAlgae =
      LEDPattern.steps(Map.of(0, LEDConstants.holdingAlgae, 1 / 3.0, LEDConstants.off));
  public LEDPattern holdingCoral =
      LEDPattern.steps(Map.of(0, LEDConstants.holdingCoral, 1 / 3.0, LEDConstants.off));

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

  public LED(ScoringSubsystem scoringSubsystem, ClimbSubsystem climbSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
    this.climbSubsystem = climbSubsystem;
    led.setLength(LEDConstants.totalLength);
    led.start();
  }

  @Override
  public void periodic() {
    clear();

    if (!enabled) {
        //if not enabled LEDs are left cleared
    } else if (DriverStation.isDisabled()) {
      if (ModeConstants.currentMode == Mode.REAL) {
        rainbow();
      }
    } else {
      if ((scoringSubsystem != null && scoringSubsystem.getGamePiece() == GamePiece.Algae)) {
        runSplitPattern(holdingAlgae, clear);
      }
    }
    
    if (ModeConstants.currentMode == Mode.REAL) {
      led.setData(ledStrip);
    }
  }

  public void addPattern(LEDPattern left, LEDPattern right) {
    leftPatterns.add(left);
    rightPatterns.add(right);
  }

  public void clear() {
    for (int i = 0; i < leftData.getLength(); i++) {
      leftData.setRGB(i, 0, 0, 0);
    }
    for (int i = 0; i < rightData.getLength(); i++) {
      rightData.setRGB(i, 0, 0, 0);
    }
  }

  public void rainbow() {
    runPattern(rainbow);
  }

  public Command runPattern(LEDPattern pattern) {
    return runSplitPattern(pattern, pattern);
  }

  /** Runs separate patterns on the top and bottom LED sections. */
  public Command runSplitPattern(LEDPattern up, LEDPattern down) {
    return run(
        () -> {
          down.applyTo(leftData);
          up.applyTo(rightData);
          mergeBuffers(); // Ensure buffer updates properly
        });
  }

  public void applyPatterns() {
    // Start with a base pattern (off/black)
    LEDPattern leftFinal = LEDPattern.solid(Color.kBlack);
    LEDPattern rightFinal = LEDPattern.solid(Color.kBlack);

    // Overlay all active patterns
    for (LEDPattern pattern : leftPatterns) {
      leftFinal = pattern.overlayOn(leftFinal);
    }
    for (LEDPattern pattern : rightPatterns) {
      rightFinal = pattern.overlayOn(rightFinal);
    }

    // Apply patterns to the left and right views
    leftFinal.applyTo(leftData);
    rightFinal.applyTo(rightData);

    // Merge views into the full buffer
    mergeBuffers();
  }

  private void mergeBuffers() {
    int halfLength = leftData.getLength();

    for (int i = 0; i < halfLength; i++) {
      ledStrip.setRGB(i, leftData.getRed(i), leftData.getGreen(i), leftData.getBlue(i));
      ledStrip.setRGB(
          i + halfLength, rightData.getRed(i), rightData.getGreen(i), rightData.getBlue(i));
    }
  }

  public Command runCycle() {
    // Just makes sure LEDs aren't destroying themselves or something
    return new Command() {
      private double lastTime;
      private int currentIndex = 0;

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
        runSplitPattern(patterns.get(currentIndex), patterns.get(currentIndex)).schedule();
      }

      @Override
      public void execute() {
        double now = Timer.getFPGATimestamp();
        if (now - lastTime >= 5.5) { // One second elapsed
          currentIndex = (currentIndex + 1) % patterns.size();
          if (currentIndex < patterns.size()) {
            // Schedule the next color.
            runSplitPattern(
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
