package frc.robot.constants;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

public final class LEDConstants {

  public static final int ledPort = 9;
  public static final int halfLength = 14; // alternate number to test = 504
  public static final int totalLength = 28; // alternate number to test = 1008
  public static final double rainbowSpeed = 50;

  public static final Color off = Color.kBlack;
  // LED Patterns

  // all thirds
  public static LEDPattern rainbowPattern =
      LEDPattern.rainbow(255, 255)
          .scrollAtRelativeSpeed(Percent.per(Second).of(LEDConstants.rainbowSpeed));
  public LEDPattern endGamePattern = LEDPattern.solid(Color.kRed);

  public static LEDPattern lockedOnHangPattern = LEDPattern.solid(Color.kTeal);

  public static LEDPattern clear = LEDPattern.solid(off);

  public static LEDPattern lasers =
      LEDPattern.gradient(GradientType.kContinuous, Color.kCrimson, Color.kBlack)
          .scrollAtAbsoluteSpeed(InchesPerSecond.of(250), Meters.of(1.0 / 60));
  // top third
  public static LEDPattern holdingAlgaePattern =
      LEDPattern.steps(Map.of(0, Color.kLimeGreen, 1 / 3.0, LEDConstants.off));

  public static LEDPattern holdingCoralPattern =
      LEDPattern.steps(Map.of(0, Color.kMagenta, 1 / 3.0, LEDConstants.off));

  public static LEDPattern holdingBothPattern =
      LEDPattern.steps(Map.of(0, Color.kWhite, 1 / 3.0, LEDConstants.off));
  public static LEDPattern clearTop = LEDPattern.steps(Map.of(1 / 3.0, Color.kBlack));
  // middle third & bottom third
  public static LEDPattern targetOnReefL1Pattern = LEDPattern.steps(Map.of(1 / 3.0, Color.kCyan));

  public static LEDPattern targetOnReefOTF = LEDPattern.steps(Map.of(1 / 3.0, Color.kBlue));

  public static LEDPattern targetOnReefL2Pattern =
      LEDPattern.steps(Map.of(1 / 3.0, Color.kOrangeRed));

  public static LEDPattern targetOnReefL3Pattern = LEDPattern.steps(Map.of(1 / 3.0, Color.kPurple));

  public static LEDPattern targetOnReefL4Pattern =
      LEDPattern.steps(Map.of(1 / 3.0, Color.kGreenYellow));

  public LEDPattern targetOnCoralStation = LEDPattern.steps(Map.of(1 / 3.0, Color.kYellow));

  // bottom third
  public static LEDPattern isBeeLinkWorkingPattern =
      LEDPattern.steps(Map.of(0, LEDConstants.off, 2 / 3.0, Color.kBlue));
}
