package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  // Configuration constants: adjust as needed for your hardware.
  private static final int LED_COUNT = 504; // Total number of LEDs (should be divisible by 3)
  private static final int PWM_PORT = 9; // PWM port for the LED strip

  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;

  // Split the LED strip into three sections.
  private final AddressableLEDBufferView leftData;
  private final AddressableLEDBufferView middleData;
  private final AddressableLEDBufferView rightData;

  // A default scrolling rainbow pattern for full-strip use.
  public static final LEDPattern rainbow =
      LEDPattern.rainbow(255, 10000).scrollAtRelativeSpeed(Frequency.ofBaseUnits(100, Hertz));

  /** Constructs the LED subsystem, splitting the LED strip into three equal parts. */
  public LED() {
    led = new AddressableLED(PWM_PORT);
    ledBuffer = new AddressableLEDBuffer(LED_COUNT);
    led.setLength(LED_COUNT);

    // Calculate the size of one third.
    int third = LED_COUNT / 3;

    // Create views for each third.
    leftData = ledBuffer.createView(0, third - 1);
    middleData = ledBuffer.createView(third, 2 * third - 1);
    rightData = ledBuffer.createView(2 * third, LED_COUNT - 1);

    // Start with an empty (off) pattern.
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    led.setData(ledData);
  }

  public void enabled(boolean enabled) {
    if (enabled) {
      led.start();
    } else {
      led.stop();
    }
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
