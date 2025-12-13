package frc.robot.subsystems.copperarm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.profile.MotionProfileConfig;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.JsonConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** A subsystem using coppercore's SparkMax IOs to control an arm. */
public class CopperarmSubsystem extends MonitoredSubsystem {
  MotorInputsAutoLogged leadMotorInputs = new MotorInputsAutoLogged();
  MotorInputsAutoLogged followerMotorInputs = new MotorInputsAutoLogged();

  MotorIO leadMotor;
  MotorIO followerMotor;

  @AutoLogOutput(key = "Copperarm/goalAngle")
  double currentGoalAngleRadians = 0.0;

  public CopperarmSubsystem(MotorIO leaderIO, MotorIO followerIO) {
    this.leadMotor = leaderIO;
    this.followerMotor = followerIO;

    leadMotor.setProfileConstraints(
        MotionProfileConfig.immutable(
            RotationsPerSecond.of(
                JsonConstants.wristConstants.wristMotionMagicCruiseVelocityRotationsPerSecond),
            RotationsPerSecondPerSecond.of(12 / JsonConstants.wristConstants.wristKA),
            RotationsPerSecondPerSecond.zero().div(Seconds.of(1.0)),
            Volts.of(JsonConstants.wristConstants.wristMotionMagicExpo_kV)
                .div(RotationsPerSecond.of(1.0)),
            Volts.of(JsonConstants.wristConstants.wristMotionMagicExpo_kA)
                .div(RotationsPerSecondPerSecond.of(1.0))));
  }


  @Override
  public void monitoredPeriodic() {
    // "subsystem" duties
    if (DriverStation.isEnabled()
            && Math.abs(leadMotorInputs.positionRadians - currentGoalAngleRadians) < 0.03) {
        currentGoalAngleRadians = Math.random() * JsonConstants.wristConstants.wristMaxMaxAngle.minus(JsonConstants.wristConstants.wristMinMinAngle).in(Radians) - JsonConstants.wristConstants.wristMinMinAngle.in(Radians);
    }

    // "mechanism" duties
    leadMotor.controlToPositionProfiled(Radians.of(currentGoalAngleRadians));
    leadMotor.updateInputs(leadMotorInputs);
    followerMotor.updateInputs(followerMotorInputs);

    Logger.processInputs("copperarm/leadMotorInputs", leadMotorInputs);
    Logger.processInputs("copperarm/followerMotorInputs", followerMotorInputs);
  }
}