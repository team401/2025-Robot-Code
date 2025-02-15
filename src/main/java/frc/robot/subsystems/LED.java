package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LED extends SubsystemBase {

  private final AddressableLED led = new AddressableLED(0);
  private final AddressableLEDBuffer ledData = new AddressableLEDBuffer(60);

  private final AddressableLEDBufferView leftData =
      ledData.createView(0, LEDConstants.leftLength - 1);
  private final AddressableLEDBufferView rightData =
      ledData.createView(LEDConstants.leftLength, LEDConstants.rightLength - 1).reversed();

  public static final LEDPattern rainbow =
      LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Frequency.ofBaseUnits(0.5, Hertz));

  public LED() {
    led.setLength(LEDConstants.leftLength + LEDConstants.rightLength);
    led.start();
  }

  @Override
  public void periodic() {
    led.setData(ledData);
  }

  public Command run(LEDPattern pattern) {
    return runSplitPatterns(pattern, pattern);
  }

  public Command runSplitPatterns(LEDPattern left, LEDPattern right) {
    return run(
        () -> {
          left.applyTo(leftData);
          right.applyTo(rightData);
        });
  }
}
