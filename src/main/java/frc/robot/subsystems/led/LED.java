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
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

/** LED subsystem for controlling addressable LEDs on the robot. */
public class LED extends SubsystemBase {

  private List<LEDPattern> leftPatterns = new ArrayList<>();
  private List<LEDPattern> rightPatterns = new ArrayList<>();

  private final AddressableLED led = new AddressableLED(LEDConstants.ledPort);
  public final AddressableLEDBuffer ledStrip = new AddressableLEDBuffer(LEDConstants.totalLength);

  public final AddressableLEDBufferView leftData =
      ledStrip.createView(0, LEDConstants.leftLength - 1);
  private final AddressableLEDBufferView rightData =
      ledStrip.createView(LEDConstants.leftLength, LEDConstants.totalLength - 1).reversed();

  // Predefined LED patterns
  public LEDPattern rainbow =
      LEDPattern.rainbow(255, 255)
          .scrollAtRelativeSpeed(Percent.per(Second).of(LEDConstants.rainbowSpeed));
  public LEDPattern holdingAlgae =
      LEDPattern.steps(Map.of(0, LEDConstants.holdingAlgae, 1 / 3.0, LEDConstants.off));
  public LEDPattern holdingCoral =
      LEDPattern.steps(Map.of(0, LEDConstants.holdingCoral, 1 / 3.0, LEDConstants.off));
  public LEDPattern targetOnReefL1 = LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnReefL1));
  public LEDPattern targetOnReefOTF = LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnReef));
  public LEDPattern targetOnReefL2 = LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnReefL2));
  public LEDPattern targetOnReefL3 = LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnReefL3));
  public LEDPattern targetOnReefL4 = LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnReefL4));
  public LEDPattern targetOnCoralStation =
      LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnCoral));
  public LEDPattern endGame = LEDPattern.solid(Color.kRed);
  public LEDPattern lockedOnHang = LEDPattern.solid(LEDConstants.lockedOnHang);
  public LEDPattern clear = LEDPattern.solid(Color.kBlack);

  private ScoringSubsystem scoringSubsystem;
  private ClimbSubsystem climbSubsystem;
  private Drive driveSubsystem;

  /**
   * Constructs the LED subsystem.
   *
   * @param scoringSubsystem The scoring subsystem
   * @param climbSubsystem The climb subsystem
   * @param drive The drive subsystem
   */
  public LED(ScoringSubsystem scoringSubsystem, ClimbSubsystem climbSubsystem, Drive drive) {
    this.scoringSubsystem = scoringSubsystem;
    this.climbSubsystem = climbSubsystem;
    this.driveSubsystem = drive;
    led.setLength(LEDConstants.totalLength);
    led.start();
  }

  /** Periodic method called every loop cycle. Updates LED patterns based on robot state. */
  @Override
  public void periodic() {
    if (!DriverStation.isDisabled()) {
      if (scoringSubsystem != null && scoringSubsystem.getGamePiece() == GamePiece.Algae) {
        addSplitPattern(holdingAlgae, clear);
      }
      if (scoringSubsystem != null && scoringSubsystem.getGamePiece() == GamePiece.Coral) {
        addSplitPattern(clear, holdingCoral);
      }
      if (scoringSubsystem != null && scoringSubsystem.getTarget() == FieldTarget.L1) {
        addSplitPattern(targetOnReefL1, clear);
      }
      if (scoringSubsystem != null && scoringSubsystem.getTarget() == FieldTarget.L2) {
        addSplitPattern(targetOnReefL2, clear);
      }
      if (scoringSubsystem != null && scoringSubsystem.getTarget() == FieldTarget.L3) {
        addSplitPattern(targetOnReefL3, clear);
      }
      if (scoringSubsystem != null && scoringSubsystem.getTarget() == FieldTarget.L4) {
        addSplitPattern(targetOnReefL4, clear);
      }
      if (climbSubsystem != null && climbSubsystem.getLockedToCage()) {
        addPattern(lockedOnHang);
      }
      if (driveSubsystem != null && driveSubsystem.isDesiredLocationReef()) {
        addSplitPattern(clear, targetOnReefOTF);
      }
      applyPatterns();
    } else {
      runPattern(rainbow);
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
  }

  public Command runCycle() {
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
        runSplitPattern(
            patterns.get(currentIndex), patterns.get(currentIndex)); // Apply first pattern
      }

      @Override
      public void execute() {
        double now = Timer.getFPGATimestamp();
        if (now - lastTime >= 5.5) { // Wait 5.5 seconds before switching
          currentIndex = (currentIndex + 1) % patterns.size();
          runSplitPattern(
              patterns.get(currentIndex), patterns.get(currentIndex)); // Apply next pattern
          lastTime = now;
        }
      }

      @Override
      public boolean isFinished() {
        return false; // Keeps running indefinitely
      }

      @Override
      public void end(boolean interrupted) {
        // Optionally clear the LEDs or stop cycling when the command ends
        runPattern(clear);
      }
    };
  }
}
