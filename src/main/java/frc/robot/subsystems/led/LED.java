package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
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
  private ClimbSubsystem climbSubsystem;
  private Drive driveSubsystem;

  private boolean enabled = false;

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

  private void setEnabled(boolean enabled) {
    if (enabled != this.enabled) {
      this.enabled = enabled;
      if (enabled) {
        ledStrip = new AddressableLEDBuffer(LEDConstants.totalLength);
        leftData = ledStrip.createView(0, LEDConstants.halfLength - 1);
        rightData =
            ledStrip.createView(LEDConstants.halfLength, LEDConstants.totalLength - 1).reversed();
        led.setLength(LEDConstants.totalLength);
      } else {
        ledStrip = new AddressableLEDBuffer(LEDConstants.totalLengthRainbow);
        led.setLength(LEDConstants.totalLengthRainbow);
      }
    }
  }

  /** Periodic method called every loop cycle. Updates LED patterns based on robot state. */
  @Override
  public void periodic() {
    if (!DriverStation.isDisabled()) {

      // Drive Subsystem Checks
      // otf - bottom middle third of left strip
      if (driveSubsystem != null) {
        if (driveSubsystem.isDesiredLocationReef()) {
          addSplitPattern(LEDConstants.targetOnReefOTF, LEDConstants.clear);
        }
      }
      // Scoring Subsystem Checks
      if (scoringSubsystem != null) {
        // Game Piece Checks - top thirds
        if (scoringSubsystem.getGamePiece() == GamePiece.Algae) {
          addPattern(LEDConstants.holdingAlgaePattern);
        } else if (scoringSubsystem.getGamePiece() == GamePiece.Coral) {
          addPattern(LEDConstants.holdingCoralPattern);
        } else if (scoringSubsystem.getGamePiece() == GamePiece.Coral
            && scoringSubsystem.getGamePiece() == GamePiece.Algae) {
          addPattern(LEDConstants.holdingBothPattern);
        } else {
          addPattern(LEDConstants.clearTop);
        }
        // Field Target Checks - bottom and middle thirds
        if (scoringSubsystem.getCoralTarget() == FieldTarget.L1) {
          addPattern(LEDConstants.targetOnReefL1Pattern);
        } else if (scoringSubsystem.getCoralTarget() == FieldTarget.L2) {
          addPattern(LEDConstants.targetOnReefL2Pattern);
        } else if (scoringSubsystem.getCoralTarget() == FieldTarget.L3) {
          addPattern(LEDConstants.targetOnReefL3Pattern);
        } else if (scoringSubsystem.getCoralTarget() == FieldTarget.L4) {
          addPattern(LEDConstants.targetOnReefL4Pattern);
        }
      }

      // check vision - bottom thirds
      if (!visionWorkingSupplier.get()) {
        addPattern(LEDConstants.isBeeLinkWorkingPattern);
      }

      // Hang Check - all thirds
      if (climbSubsystem != null) {
        if (climbSubsystem.getLockedToCage()) {
          addPattern(LEDConstants.lockedOnHangPattern);
        }
      }

      applyPatterns();

    }
    // lasers(disabled) - all thirds
    // else if (driveSubsystem.isBrakeMode()) {
    //  addPattern(LEDConstants.lasers);
    // }
    else {
      // rainbow - all thirds
      // addPattern(LEDConstants.rainbowPattern);
      // if (!visionWorkingSupplier.get()) {
      //   addPattern(LEDConstants.isBeeLinkWorkingPattern);
      // }
      // applyPatterns();
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
