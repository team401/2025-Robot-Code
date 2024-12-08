package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim extends ElevatorIOTalonFX {
    private final ElevatorSim elevatorSim =
            new ElevatorSim(
                    DCMotor.getKrakenX60Foc(4),
                    ElevatorConstants.elevatorReduction,
                    ElevatorConstants.carriageMass.in(Kilograms),
                    ElevatorConstants.drumRadius.in(Meters),
                    0.0,
                    2.0,
                    true,
                    // Seed to a random position to test CRT
                    2.0);

    public ElevatorIOSim() {
        super();

        // Initialize sim state properly so CRT can seed properly
        updateSimState();
    }

    private void updateSimState() {
        Distance elevatorHeight = Meters.of(elevatorSim.getPositionMeters());
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

        largeCANCoder.getSimState().setRawPosition(largeEncoderRotations);
        smallCANCoder.getSimState().setRawPosition(smallEncoderRotations);

        TalonFXSimState leadMotorSim = leadMotor.getSimState();
        TalonFXSimState followerMotorSim = followerMotor.getSimState();

        leadMotorSim.setRawRotorPosition(motorRotations);
        followerMotorSim.setRawRotorPosition(motorRotations);

        leadMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        followerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInputVoltage(leadMotorSim.getMotorVoltage());
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        updateSimState();

        // Assume 50hz
        elevatorSim.update(0.020);
        super.updateInputs(inputs);
    }
}
