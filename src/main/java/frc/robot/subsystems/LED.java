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
    // Continuously update the LED strip with the current buffer data.
    led.setData(ledBuffer);
  }

  /** Returns a command that continuously applies the rainbow pattern to the entire LED strip. */
  public Command getRainbowCommand() {
    return new Command() {
      {
        addRequirements(LED.this);
      }

      @Override
      public void execute() {
        // Apply the rainbow pattern across the entire strip.
        rainbow.applyTo(ledBuffer);
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };
  }

  /**
   * Returns a command that continuously applies distinct patterns to each third of the LED strip.
   * Use this to "send messages" by displaying different effects on the left, middle, and right
   * sections.
   *
   * @param leftPattern The pattern for the left third.
   * @param middlePattern The pattern for the middle third.
   * @param rightPattern The pattern for the right third.
   * @return a command that applies these patterns continuously.
   */
  public Command getMessageCommand(
      LEDPattern leftPattern, LEDPattern middlePattern, LEDPattern rightPattern) {
    return new Command() {
      {
        addRequirements(LED.this);
      }

      @Override
      public void execute() {
        leftPattern.applyTo(leftData);
        middlePattern.applyTo(middleData);
        rightPattern.applyTo(rightData);
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };
  }

  // Optionally, helper methods to apply patterns to individual sections:

  public void runPatternOnLeft(LEDPattern pattern) {
    pattern.applyTo(leftData);
  }

  public void runPatternOnMiddle(LEDPattern pattern) {
    pattern.applyTo(middleData);
  }

  public void runPatternOnRight(LEDPattern pattern) {
    pattern.applyTo(rightData);
  }
}
