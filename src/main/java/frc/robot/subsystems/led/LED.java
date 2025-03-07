package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/** LED subsystem for controlling addressable LEDs on the robot. */
public class LED extends SubsystemBase {

  private List<LEDPattern> leftPatterns = new ArrayList<>();
  private List<LEDPattern> rightPatterns = new ArrayList<>();

  private final AddressableLED led = new AddressableLED(LEDConstants.ledPort);
  public final AddressableLEDBuffer ledStrip = new AddressableLEDBuffer(LEDConstants.totalLength);

  public AddressableLEDBufferView leftData =
      ledStrip.createView(0, LEDConstants.halfLength - 1).reversed();
  private AddressableLEDBufferView rightData =
      ledStrip.createView(LEDConstants.halfLength, LEDConstants.totalLength - 1).reversed();

  private Supplier<Boolean> visionWorkingSupplier = () -> true;
  private static final double commandWaitTime = 2;
  private ScoringSubsystem scoringSubsystem;
  private ClimbSubsystem climbSubsystem;
  private Drive driveSubsystem;

  private boolean enabled = true;

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

    if (!DriverStation.isDisabled() && !DriverStation.isTest()) {

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
        if (scoringSubsystem.isAlgaeDetected()) {
          addPattern(LEDConstants.holdingAlgaePattern);
        } else if (scoringSubsystem.isCoralDetected()) {
          addPattern(LEDConstants.holdingCoralPattern);
        } else if (scoringSubsystem.isAlgaeDetected() && scoringSubsystem.isCoralDetected()) {
          addPattern(LEDConstants.holdingBothPattern);
        } else {
          addPattern(LEDConstants.clearTop);
        }
        // Field Target Checks - bottom and middle thirds
        if (scoringSubsystem.getTarget() == FieldTarget.L1) {
          addPattern(LEDConstants.targetOnReefL1Pattern);
        } else if (scoringSubsystem.getTarget() == FieldTarget.L2) {
          addPattern(LEDConstants.targetOnReefL2Pattern);
        } else if (scoringSubsystem.getTarget() == FieldTarget.L3) {
          addPattern(LEDConstants.targetOnReefL3Pattern);
        } else if (scoringSubsystem.getTarget() == FieldTarget.L4) {
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
    else if (!DriverStation.isTest()) {

      // rainbow - all thirds
      addPattern(LEDConstants.rainbowPattern);
      if (!visionWorkingSupplier.get()) {
        addPattern(LEDConstants.isBeeLinkWorkingPattern);
      }
      applyPatterns();
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


  public void enabled(boolean enabled) {
    this.enabled = enabled;
    if (enabled) {
      led.start();
    } else {
      led.stop();
    }
  }

  public Command LEDTest() {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              runPattern(LEDConstants.clear);
              led.setData(ledStrip);
            }),
        new WaitCommand(commandWaitTime),
        new InstantCommand(
            () -> {
              runPattern(LEDConstants.holdingAlgaePattern);
              led.setData(ledStrip);
            }),
        new WaitCommand(commandWaitTime),
        new InstantCommand(
            () -> {
              runPattern(LEDConstants.holdingCoralPattern);
              led.setData(ledStrip);
            }),
        new WaitCommand(commandWaitTime),
        new InstantCommand(
            () -> {
              runPattern(LEDConstants.holdingBothPattern);
              led.setData(ledStrip);
            }),
        new WaitCommand(commandWaitTime),
        new InstantCommand(
            () -> {
              runPattern(LEDConstants.targetOnReefL1Pattern);
              led.setData(ledStrip);
            }),
        new WaitCommand(commandWaitTime),
        new InstantCommand(
            () -> {
              runPattern(LEDConstants.targetOnReefL2Pattern);
              led.setData(ledStrip);
            }),
        new WaitCommand(commandWaitTime),
        new InstantCommand(
            () -> {
              runPattern(LEDConstants.targetOnReefL3Pattern);
              led.setData(ledStrip);
            }),
        new WaitCommand(commandWaitTime),
        new InstantCommand(
            () -> {
              runPattern(LEDConstants.targetOnReefL4Pattern);
              led.setData(ledStrip);
            }),
        new WaitCommand(commandWaitTime),
        new InstantCommand(
            () -> {
              runPattern(LEDConstants.isBeeLinkWorkingPattern);
              led.setData(ledStrip);
            }),
        new WaitCommand(commandWaitTime));
  }
}
