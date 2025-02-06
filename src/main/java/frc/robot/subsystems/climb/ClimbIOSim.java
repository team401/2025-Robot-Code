package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.ClimbConstants;
import org.littletonrobotics.junction.Logger;

public class ClimbIOSim extends ClimbIOTalonFX {

  TalonFXSimState leadMotorSimState = leadMotor.getSimState();
  TalonFXSimState followerMotorSimState = followerMotor.getSimState();

  private SingleJointedArmSim climb =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          ClimbConstants.Sim.synced.getObject().climbGearing,
          SingleJointedArmSim.estimateMOI(
              ClimbConstants.Sim.synced.getObject().climbArmLengthMeters,
              ClimbConstants.Sim.synced.getObject().climbArmMassKg),
          ClimbConstants.Sim.synced.getObject().climbArmLengthMeters,
          0,
          2 * Math.PI,
          false,
          0,
          ClimbConstants.Sim.synced.getObject().climbStdDevs,
          0);

  public ClimbIOSim() {
    super();
  }

  @Override
  public void updateInputs(ClimbInputs inputs) {
    leadMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    followerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

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
