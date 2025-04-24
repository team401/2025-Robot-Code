package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.SimConstants;
import frc.robot.constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class WristIOSim extends WristIOTalonFX {
  CANcoderSimState wristEncoderSimState = wristEncoder.getSimState();

  TalonFXSimState wristMotorSimState = wristMotor.getSimState();

  private final SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          WristConstants.synced.getObject().wristReduction,
          WristConstants.Sim.synced.getObject().wristMomentOfInertia.in(KilogramSquareMeters),
          WristConstants.Sim.synced.getObject().wristArmLength.in(Meters),
          WristConstants.Sim.synced.getObject().wristMinAngle.in(Radians),
          WristConstants.Sim.synced.getObject().wristMaxAngle.in(Radians),
          true,
          WristConstants.Sim.synced.getObject().wristStartingAngle.in(Radians));

  MutAngle lastWristAngle = Radians.mutable(0.0);

  public WristIOSim() {
    super();

    wristEncoderSimState.Orientation = ChassisReference.Clockwise_Positive;

    // Initialize sim state so that the first periodic runs with accurate data
    updateSimState();
  }

  private void updateSimState() {
    Angle wristAngle = Radians.of(wristSim.getAngleRads());
    AngularVelocity wristVelocity = RadiansPerSecond.of(wristSim.getVelocityRadPerSec());

    Angle diffAngle = wristAngle.minus(lastWristAngle);
    lastWristAngle.mut_replace(wristAngle);

    // 1:1 ratio of Wrist to CANcoder makes this math very easy
    wristEncoderSimState.setRawPosition(
        wristAngle.minus(
            WristConstants.synced.getObject()
                .wristEncoderMagnetOffset)); // Subtract the magnet offset since it's 0 in sim
    wristEncoderSimState.setVelocity(wristVelocity);

    Angle rotorDiffAngle = diffAngle.times(WristConstants.synced.getObject().wristReduction);
    AngularVelocity rotorVelocity =
        wristVelocity.times(WristConstants.synced.getObject().wristReduction);
    wristMotorSimState.addRotorPosition(rotorDiffAngle);
    wristMotorSimState.setRotorVelocity(rotorVelocity);
    wristMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    wristSim.setInputVoltage(wristMotorSimState.getMotorVoltage());

    Logger.recordOutput("wristSim/position", wristAngle.in(Radians));

    wristSim.update(SimConstants.simDeltaTime.in(Seconds));
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    updateSimState();

    super.updateInputs(inputs);
  }
}
