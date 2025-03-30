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
  public static final int halfLength = 28;
  public static final int totalLength = 56;

  public static final Color off = Color.kBlack;
  // LED Patterns

  // all thirds
  public static LEDPattern rainbowPattern =
      LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(20));

  public static LEDPattern clear = LEDPattern.solid(off);

  public static LEDPattern endGame =
      LEDPattern.gradient(GradientType.kContinuous, Color.kBlue, Color.kBlack)
          .scrollAtAbsoluteSpeed(InchesPerSecond.of(250), Meters.of(1.0 / 60));

  public static LEDPattern holdingBothPattern = LEDPattern.solid(Color.kYellow);
  public static LEDPattern holdingCoralPattern = LEDPattern.solid(Color.kMagenta);
  public static LEDPattern holdingAlgaePattern = LEDPattern.solid(Color.kLime);

  // bottom third
  public static LEDPattern isBeeLinkWorkingPattern =
      LEDPattern.steps(Map.of(0, LEDConstants.off, 2 / 3.0, Color.kBlue));
}
