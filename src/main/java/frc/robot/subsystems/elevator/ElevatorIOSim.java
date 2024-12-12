package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim extends ElevatorIOTalonFX {
    CANcoderSimState largeCANcoderSimState = largeCANCoder.getSimState();
    CANcoderSimState smallCANcoderSimState = smallCANCoder.getSimState();

    TalonFXSimState leadMotorSimState = leadMotor.getSimState();
    TalonFXSimState followerMotorSimState = followerMotor.getSimState();

    private final ElevatorSim elevatorSim =
            new ElevatorSim(
                    DCMotor.getKrakenX60Foc(4),
                    ElevatorConstants.elevatorReduction,
                    ElevatorConstants.carriageMass.in(Kilograms),
                    ElevatorConstants.drumRadius.in(Meters),
                    0.0,
                    3.0,
                    true,
                    1.0,
                    0.0,
                    0.0);

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
        AngularVelocity motorVelocity =
                RotationsPerSecond.of(
                        Meters.of(elevatorSim.getVelocityMetersPerSecond())
                                .divide(Inches.of(4.724))
                                .magnitude());

        largeCANcoderSimState.setRawPosition(largeEncoderRotations);
        smallCANcoderSimState.setRawPosition(smallEncoderRotations);

        leadMotorSimState.setRawRotorPosition(motorRotations);
        leadMotorSimState.setRotorVelocity(motorVelocity);
        followerMotorSimState.setRawRotorPosition(motorRotations);
        followerMotorSimState.setRotorVelocity(motorVelocity);

        leadMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        followerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInputVoltage(leadMotorSimState.getMotorVoltage());
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        updateSimState();

        // Assume 50hz
        elevatorSim.update(0.020);
        super.updateInputs(inputs);
    }
}
