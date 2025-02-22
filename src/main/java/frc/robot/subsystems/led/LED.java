package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

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
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class LED extends SubsystemBase {

  private List<LEDPattern> leftPatterns = new ArrayList<>();
  private List<LEDPattern> rightPatterns = new ArrayList<>();

  private final AddressableLED led = new AddressableLED(LEDConstants.ledPort);
  public final AddressableLEDBuffer ledStrip = new AddressableLEDBuffer(LEDConstants.totalLength);

  public final AddressableLEDBufferView leftData =
      ledStrip.createView(0, LEDConstants.leftLength - 1);
  private final AddressableLEDBufferView rightData =
      ledStrip.createView(LEDConstants.leftLength, LEDConstants.totalLength - 1).reversed();

  public LEDPattern rainbow =
      LEDPattern.rainbow(255, 255)
          .scrollAtRelativeSpeed(Percent.per(Second).of(LEDConstants.rainbowSpeed));
  public LEDPattern rainbow1 =
      LEDPattern.rainbow(255, 255)
          .scrollAtRelativeSpeed(Percent.per(Second).of(LEDConstants.rainbowSpeed));
  public LEDPattern holdingAlgae =
      LEDPattern.steps(Map.of(0, LEDConstants.holdingAlgae, 1 / 3.0, LEDConstants.off));
  public LEDPattern holdingCoral =
      LEDPattern.steps(Map.of(0, LEDConstants.holdingCoral, 1 / 3.0, LEDConstants.off));

  public LEDPattern targetOnReef = LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnReef));

  public LEDPattern targetOnReefL1 = LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnReefL1));

  public LEDPattern targetOnReefL2 = LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnReefL2));

  public LEDPattern targetOnReefL3 = LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnReefL3));

  public LEDPattern targetOnReefL4 = LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnReefL4));

  public LEDPattern targetOnProcessor =
      LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnProcessor));

  public LEDPattern targetOnNet = LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnNet));

  public LEDPattern targetOnCoralStation =
      LEDPattern.steps(Map.of(1 / 3.0, LEDConstants.targetOnCoral));
  public LEDPattern endGame = LEDPattern.solid(Color.kRed);

  public LEDPattern lockedOnHang = LEDPattern.solid(LEDConstants.lockedOnHang);

  public LEDPattern clear = LEDPattern.solid(Color.kBlack);
  private ScoringSubsystem scoringSubsystem;
  private ClimbSubsystem climbSubsystem;
  private Drive driveSubsystem;

  /*
   * How LED's work:
   * 1. Add a list of colors in LEDConstants.java
   * 2. Add a list of LEDPatterns that use those colors
   * 3. LEDPatterns are where the magic happens, make your specific colors/animations/etc there
   * 4. Call them in periodic();
   */

  public LED(ScoringSubsystem scoringSubsystem, ClimbSubsystem climbSubsystem, Drive drive) {
    this.scoringSubsystem = scoringSubsystem;
    this.climbSubsystem = climbSubsystem;
    this.driveSubsystem = drive;
    led.setLength(LEDConstants.totalLength);
    led.start();
  }

  @Override
  public void periodic() {
    bufferification();
    led.setData(ledStrip);
  }

  public void bufferification() { // TODO: make this a real method name
    if (!DriverStation.isDisabled()) {

      if (scoringSubsystem != null && (scoringSubsystem.getGamePiece() == GamePiece.Algae)) {
        addSplitPattern(holdingAlgae, clear);
      }
      // coral
      if (scoringSubsystem != null && (scoringSubsystem.getGamePiece() == GamePiece.Coral)) {
        addSplitPattern(clear, holdingCoral);
      }
      // height target
      if (scoringSubsystem != null && (scoringSubsystem.getTarget() == FieldTarget.L1)) {
        addSplitPattern(targetOnReefL1, clear);
      }
      if (scoringSubsystem != null && (scoringSubsystem.getTarget() == FieldTarget.L2)) {
        addSplitPattern(targetOnReefL2, clear);
      }
      if (scoringSubsystem != null && (scoringSubsystem.getTarget() == FieldTarget.L3)) {
        addSplitPattern(targetOnReefL3, clear);
      }
      if (scoringSubsystem != null && (scoringSubsystem.getTarget() == FieldTarget.L4)) {
        addSplitPattern(targetOnReefL4, clear);
      }
      if (scoringSubsystem != null && (scoringSubsystem.getTarget() == FieldTarget.L4)) {
        addSplitPattern(targetOnReefL4, clear);
      }
      if (climbSubsystem != null && (climbSubsystem.getLockedToCage()) == true) {
        addPattern(lockedOnHang);
      }
      if (driveSubsystem != null && (driveSubsystem.isDesiredLocationReef())) {
        addSplitPattern(clear, targetOnReef);
      }
      if (driveSubsystem != null
          && (driveSubsystem.getDesiredLocation()) == (DesiredLocation.CoralStationLeft)) {
        addSplitPattern(targetOnCoralStation, clear);
      }
      if (driveSubsystem != null
          && (driveSubsystem.getDesiredLocation()) == (DesiredLocation.CoralStationRight)) {
        addSplitPattern(clear, targetOnCoralStation);
      }

      applyPatterns();
    } else {
      runPattern(rainbow);
    }
  }

  public void runSplitPattern(LEDPattern up, LEDPattern down) {
    down.applyTo(leftData);
    up.applyTo(rightData);
  }

  public void runPattern(LEDPattern pattern) {
    runSplitPattern(pattern, pattern);
  }

  public void addSplitPattern(LEDPattern left, LEDPattern right) {
    leftPatterns.add(left);
    rightPatterns.add(right);
  }

  public void addPattern(LEDPattern pattern) {
    leftPatterns.add(pattern);
    rightPatterns.add(pattern);
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
  }
}
