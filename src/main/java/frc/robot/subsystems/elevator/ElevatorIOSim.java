package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.SimConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim extends ElevatorIOTalonFX {
    CANcoderSimState largeCANcoderSimState = largeCANCoder.getSimState();
    CANcoderSimState smallCANcoderSimState = smallCANCoder.getSimState();

    TalonFXSimState leadMotorSimState = leadMotor.getSimState();
    TalonFXSimState followerMotorSimState = followerMotor.getSimState();

    private final ElevatorSim elevatorSim =
            new ElevatorSim(
                    DCMotor.getKrakenX60Foc(2),
                    ElevatorConstants.elevatorReduction,
                    ElevatorConstants.carriageMass.in(Kilograms),
                    ElevatorConstants.drumRadius.in(Meters),
                    ElevatorConstants.minElevatorHeight.in(Meters),
                    ElevatorConstants.maxElevatorHeight.in(Meters),
                    true,
                    ElevatorConstants.Sim.elevatorStartingHeight.in(Meters),
                    ElevatorConstants.Sim.positionStdDev,
                    ElevatorConstants.Sim.velocityStdDev);

    public ElevatorIOSim() {
        super();

        // Initialize sim state properly so CRT can seed properly
        updateSimState();
    }

    private void updateSimState() {
        Distance elevatorHeight = Meters.of(elevatorSim.getPositionMeters());
        LinearVelocity elevatorVelocity =
                MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond());

        Logger.recordOutput("elevator/simElevatorHeightMeters", elevatorHeight.in(Meters));
        Logger.recordOutput(
                "elevator/simElevatorVelocityMetersPerSec", elevatorVelocity.in(MetersPerSecond));

        // TODO: Use coppercore gear math after https://github.com/team401/coppercore/issues/52 is
        // done.

        Angle spoolRotations = Rotations.of(elevatorHeight.divide(Inches.of(4.724)).magnitude());
        Angle largeEncoderRotations =
                spoolRotations.times(
                        (double) ElevatorConstants.spoolTeeth
                                / (double) ElevatorConstants.largeCANCoderTeeth);
        Angle smallEncoderRotations =
                spoolRotations.times(
                        (double) ElevatorConstants.spoolTeeth
                                / (double) ElevatorConstants.smallCANCoderTeeth);

        Angle motorRotations = spoolRotations.times(ElevatorConstants.elevatorReduction);
        // Convert elevator velocity (m/s) into angular velocity of spool by dividing by elevator to
        // spool (m/rot) to obtain rot/s, then multiply by elevator reduction, because the motors
        // will spin [reduction] times as many times as spool.
        AngularVelocity motorVelocity =
                RotationsPerSecond.of(
                        elevatorVelocity.in(MetersPerSecond)
                                / ElevatorConstants.elevatorToSpool.in(
                                        PerUnit.combine(Meters, Rotations)));
        // .times(ElevatorConstants.elevatorReduction);
        // TODO: Find out why sim breaks when multiplying motor velocity by motor reduction

        AngularVelocity spoolVelocity =
                RotationsPerSecond.of(
                        elevatorVelocity.in(MetersPerSecond)
                                / ElevatorConstants.elevatorToSpool.in(
                                        PerUnit.combine(Meters, Rotations)));

        AngularVelocity largeEncoderVelocity =
                spoolVelocity.times(
                        (double) ElevatorConstants.spoolTeeth
                                / (double) ElevatorConstants.largeCANCoderTeeth);
        AngularVelocity smallEncoderVelocity =
                spoolVelocity.times(
                        (double) ElevatorConstants.spoolTeeth
                                / (double) ElevatorConstants.smallCANCoderTeeth);

        largeCANcoderSimState.setRawPosition(largeEncoderRotations);
        largeCANcoderSimState.setVelocity(largeEncoderVelocity);
        smallCANcoderSimState.setRawPosition(smallEncoderRotations);
        smallCANcoderSimState.setVelocity(smallEncoderVelocity);

        leadMotorSimState.setRawRotorPosition(motorRotations);
        leadMotorSimState.setRotorVelocity(motorVelocity);
        leadMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        followerMotorSimState.setRawRotorPosition(motorRotations);
        followerMotorSimState.setRotorVelocity(motorVelocity);
        followerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInputVoltage(leadMotorSimState.getMotorVoltage());

        Logger.recordOutput("elevatorSim/largeEncoderPos", largeEncoderRotations);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        updateSimState();

        // Assume 50hz
        elevatorSim.update(SimConstants.simDeltaTime.in(Seconds));
        super.updateInputs(inputs);
    }
}
