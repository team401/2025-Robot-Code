package frc.robot.subsystems.coppervator;

import org.littletonrobotics.junction.Logger;

import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.encoders.EncoderIO;
import coppercore.wpilib_interface.subsystems.encoders.EncoderInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;

/**
 * The Coppervator is a subsystem written to demonstrate coppercore's IO classes by implementing an
 * elevator.
 */
public class CoppervatorSubsystem extends MonitoredSubsystem {
  MotorInputsAutoLogged leadMotorInputs = new MotorInputsAutoLogged();
  MotorInputsAutoLogged followerMotorInputs = new MotorInputsAutoLogged();

  MotorIO leadMotor;
  MotorIO followerMotor;

  EncoderInputsAutoLogged cancoderInputs = new EncoderInputsAutoLogged();
  EncoderIO encoder;

  public CoppervatorSubsystem(MotorIO leaderIO, MotorIO followerIO, EncoderIO cancoderIO) {
    this.leadMotor = leaderIO;
    this.followerMotor = followerIO;
    this.encoder = cancoderIO;
  }

  @Override
  public void monitoredPeriodic() {
    leadMotor.updateInputs(leadMotorInputs);
    followerMotor.updateInputs(followerMotorInputs);

    encoder.updateInputs(cancoderInputs);

    Logger.processInputs("coppervator/leadMotorInputs", leadMotorInputs);
    Logger.processInputs("coppervator/followerMotorInputs", followerMotorInputs);
    Logger.processInputs("coppervator/encoderInputs", cancoderInputs);
  }
}
