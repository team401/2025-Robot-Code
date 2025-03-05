package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
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
import frc.robot.constants.JsonConstants;
import frc.robot.constants.SimConstants;
import org.littletonrobotics.junction.Logger;

public class WristIOSim extends WristIOTalonFX {
  CANcoderSimState wristCANcoderSimState = wristCANcoder.getSimState();

  TalonFXSimState wristMotorSimState = wristMotor.getSimState();

  private final SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          JsonConstants.wristConstants.wristReduction,
          JsonConstants.wristConstantsSim.wristMomentOfInertia.in(KilogramSquareMeters),
          JsonConstants.wristConstantsSim.wristArmLength.in(Meters),
          JsonConstants.wristConstantsSim.wristMinAngle.in(Radians),
          JsonConstants.wristConstantsSim.wristMaxAngle.in(Radians),
          true,
          JsonConstants.wristConstantsSim.wristStartingAngle.in(Radians));

  public WristIOSim() {
    super();

    wristCANcoderSimState.Orientation = ChassisReference.Clockwise_Positive;

    // Initialize sim state so that the first periodic runs with accurate data
    updateSimState();
  }

  MutAngle lastWristAngle = Radians.mutable(0.0);

  private void updateSimState() {
    Angle wristAngle = Radians.of(wristSim.getAngleRads());
    AngularVelocity wristVelocity = RadiansPerSecond.of(wristSim.getVelocityRadPerSec());

    Angle diffAngle = wristAngle.minus(lastWristAngle);
    lastWristAngle.mut_replace(wristAngle);

    // 1:1 ratio of wrist to CANcoder makes this math very easy
    wristCANcoderSimState.addPosition(diffAngle);
    wristCANcoderSimState.setVelocity(wristVelocity);

    Angle rotorDiffAngle = diffAngle.times(JsonConstants.wristConstants.wristReduction);
    AngularVelocity rotorVelocity =
        wristVelocity.times(JsonConstants.wristConstants.wristReduction);

    wristMotorSimState.addRotorPosition(rotorDiffAngle);
    wristMotorSimState.setRotorVelocity(rotorVelocity);
    wristMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    wristSim.setInputVoltage(wristMotorSimState.getMotorVoltage());

    Logger.recordOutput("wristSim/position", wristAngle.in(Rotations));
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    updateSimState();

    wristSim.update(SimConstants.simDeltaTime.in(Seconds));

    super.updateInputs(inputs);
  }
}
