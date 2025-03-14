package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.CANdiSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.ClimbConstants;
import org.littletonrobotics.junction.Logger;

public class ClimbIOSim extends ClimbIOTalonFX {

  TalonFXSimState leadMotorSimState = leadMotor.getSimState();
  TalonFXSimState followerMotorSimState = followerMotor.getSimState();
  // CANcoderSimState climbEncoderSimState = climbAngleCoder.getSimState();
  CANdiSimState climbEncoderSimState = climbAngleCandi.getSimState();

  private SingleJointedArmSim climb =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          ClimbConstants.Sim.synced.getObject().climbGearing,
          SingleJointedArmSim.estimateMOI(
              ClimbConstants.Sim.synced.getObject().climbArmLengthMeters,
              ClimbConstants.Sim.synced.getObject().climbArmMassKg),
          ClimbConstants.Sim.synced.getObject().climbArmLengthMeters,
          ClimbConstants.Sim.synced.getObject().minAngleRads,
          ClimbConstants.Sim.synced.getObject().maxAngleRads,
          ClimbConstants.Sim.synced.getObject().simGravity,
          ClimbConstants.Sim.synced.getObject().startAngleRads,
          ClimbConstants.Sim.synced.getObject().climbStdDevs,
          0);

  public ClimbIOSim() {
    super();
  }

  @Override
  public void updateInputs(ClimbInputs inputs) {
    leadMotorSimState.setRawRotorPosition(Radians.of(climb.getAngleRads()).times(25.0));
    leadMotorSimState.setRotorVelocity(
        RadiansPerSecond.of(climb.getVelocityRadPerSec()).times(25.0));
    leadMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    /*followerMotorSimState.setRawRotorPosition(Radians.of(climb.getAngleRads()).times(25.0));
    followerMotorSimState.setRotorVelocity(
        RadiansPerSecond.of(climb.getVelocityRadPerSec()).times(25.0));
    followerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());*/

    climbEncoderSimState.setPwm1Position(Radians.of(climb.getAngleRads()));
    climbEncoderSimState.setPwm1Velocity(RadiansPerSecond.of(climb.getVelocityRadPerSec()));

    climb.setInputVoltage(leadMotorSimState.getMotorVoltage());
    climb.update(0.02);

    super.updateInputs(inputs);
  }

  @Override
  public void applyOutputs(ClimbOutputs outputs) {
    Logger.recordOutput(
        "climb/profilesetpoint", leadMotor.getClosedLoopReference().getValueAsDouble());
    super.applyOutputs(outputs);
  }
}
