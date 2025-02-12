package frc.robot.constants;

import frc.robot.Robot;

public final class ModeConstants {
  public enum Mode {
    REAL,
    SIM,
    MAPLESIM,
    REPLAY
  }

  /**
   * When in sim, this value determines whether the robot will be in replay mode or in simulation
   * mode. The robot will automatically detect a real robot, so this value doesn't need to be
   * changed to Mode.Real when on a real robot.
   */
  public static final Mode simMode = Mode.SIM;

  public static final Mode currentMode = Robot.isReal() ? Mode.REAL : simMode;
}
