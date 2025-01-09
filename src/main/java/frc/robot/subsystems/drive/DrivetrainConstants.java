package frc.robot.subsystems.drive;

public final class DrivetrainConstants {
  public static final double TURN_CRUISE_VELOCITY = 100.0; // in rot/s
  public static final double TURN_ACCELERATION_MULTIPLIER = 10.0;
  public static final double TURN_FEEDFORWARD_kV = 0.12;
  public static final double TURN_FEEDFORWARD_kA = 0.1;

  public static final double maxLinearSpeed = 6.0;
  public static final double maxAngularSpeed = Math.PI * 2;
  public static final double joystickDeadband = 0.1;
}
