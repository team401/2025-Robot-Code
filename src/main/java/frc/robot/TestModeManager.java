package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Object to handle switching between test modes. */
public class TestModeManager {
  public enum TestMode {
    RampTuning,
    ElevatorTuning,
    ElevatorCharacterization,
    WristVoltageTuning,
    WristClosedLoopTuning,
    ClawTuning,
    ClawOvershootTuning,
    SetpointTuning,
    DriveFeedForwardCharacterization,
    DriveWheelRadiusCharacterization,
    DriveSysIdQuasistaticForward,
    DriveSteerMotorCharacterization,
    DriveSysIdQuasistaticBackward,
    DriveSysIdDynamicForward,
    DriveSysIdDynamicBackward,
    DriveLineupTuning,
    LEDTest,
    ClimbTuning,
    None, // Default test mode that does nothing until a new one is selected.
  }

  private static SendableChooser<TestMode> testModeChooser;

  /**
   * Initialize test modes by publishing the test mode chooser. This method should be called by the
   * Robot, as it doesn't run automatically
   */
  public static void init() {
    testModeChooser = new SendableChooser<TestMode>();

    testModeChooser.setDefaultOption("None", TestMode.None);
    // Ramp Test Mode
    testModeChooser.addOption("Ramp Tuning", TestMode.RampTuning);
    // Scoring Test Modes
    testModeChooser.addOption("Elevator Tuning", TestMode.ElevatorTuning);
    testModeChooser.addOption("Elevator Characterization", TestMode.ElevatorCharacterization);
    testModeChooser.addOption("Wrist Voltage Tuning", TestMode.WristVoltageTuning);
    testModeChooser.addOption("Wrist Closed-Loop Tuning", TestMode.WristClosedLoopTuning);
    testModeChooser.addOption("Claw Tuning", TestMode.ClawTuning);
    testModeChooser.addOption("Claw Overshoot Tuning", TestMode.ClawOvershootTuning);
    testModeChooser.addOption("Scoring Setpoint Tuning", TestMode.SetpointTuning);
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
    testModeChooser.addOption("Drive Lineup Tuning", TestMode.DriveLineupTuning);
    // Climb Test Modes
    testModeChooser.addOption("Climb Tuning", TestMode.ClimbTuning);

    // LED
    testModeChooser.addOption("LED Cycle Test", TestMode.LEDTest);

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
