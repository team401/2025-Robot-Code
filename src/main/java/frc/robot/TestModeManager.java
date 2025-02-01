package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Object to handle switching between test modes. */
public class TestModeManager {
  public enum TestMode {
    ElevatorTuning,
    ClawTuning,
    ClawOvershootTuning,
    DriveFeedForwardCharacterization,
    DriveWheelRadiusCharacterization,
    DriveSysIdQuasistaticForward,
    DriveSteerMotorCharacterization,
    DriveSysIdQuasistaticBackward,
    DriveSysIdDynamicForward,
    DriveSysIdDynamicBackward,
    None, // Default test mode that does nothing until a new one is selected.
  }

  private static SendableChooser<TestMode> testModeChooser;

  /**
   * Initialize test modes by publishing the test mode chooser. This method should be called by the
   * Robot, as it doesn't run automatically
   */
  public static void testInit() {
    testModeChooser = new SendableChooser<TestMode>();

    testModeChooser.setDefaultOption("None", TestMode.None);
    // Elevator Test Modes
    testModeChooser.addOption("Elevator Tuning", TestMode.ElevatorTuning);
    testModeChooser.addOption("Claw Tuning", TestMode.ClawTuning);
    // Drive Test Modes
    testModeChooser.addOption(
        "Drive FeedForward Characterization", TestMode.DriveFeedForwardCharacterization);
    testModeChooser.addOption(
        "Drive Steer Motor Characterization", TestMode.DriveSteerMotorCharacterization);
    testModeChooser.addOption(
        "Drive Wheel Radius Characterization", TestMode.DriveWheelRadiusCharacterization);
    testModeChooser.addOption(
        "Drive SysId (Quasistatic Forward)", TestMode.DriveSysIdQuasistaticForward);
    testModeChooser.addOption(
        "Drive SysId (Quasistatic Reverse)", TestMode.DriveSysIdQuasistaticBackward);
    testModeChooser.addOption("Drive SysId (Dynamic Forward)", TestMode.DriveSysIdDynamicForward);
    testModeChooser.addOption("Drive SysId (Dynamic Reverse)", TestMode.DriveSysIdDynamicBackward);

    SmartDashboard.putData("Test Mode Selector", testModeChooser);
  }

  public static TestMode getTestMode() {
    if (DriverStation.isTest()) {
      return testModeChooser.getSelected();
    } else {
      return TestMode.None;
    }
  }
}
