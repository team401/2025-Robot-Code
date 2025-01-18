package frc.robot.subsystems.climb;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimbIOSim implements ClimbIO {
    
    private boolean lockedToCage;

    private SingleJointedArmSim climb = new SingleJointedArmSim(DCMotor.getKrakenX60(2),75, SingleJointedArmSim.estimateMOI(0.75, 0.5), 0.5, 0, 90, false, 0, null);

    private final PIDController angleController =
            new PIDController(0,0,0);

    private MutVoltage goalVoltage = Volts.mutable(0.0);

    @Override
    public void updateInputs(ClimbInputs inputs) {

        inputs.lockedToCage = this.lockedToCage;
        //inputs.motorVoltage.mut_replace(Volts.of(climb.getMotorVoltage()));
        inputs.motorAngle.mut_replace(Radians.of(climb.getAngleRads()));
    }

    @Override
    public void applyOutputs(ClimbOutputs outputs) {
        outputs.goalVoltage.mut_replace(this.goalVoltage.in(Volts), Volts);
        climb.setInputVoltage(outputs.goalVoltage.in(Volt));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        goalVoltage.mut_replace(voltage);
    }

    public void setVoltage(double voltage) {
        goalVoltage.mut_replace(Volts.of(voltage));
    }

    @Override
    public void setPID(double p, double i, double d) {
        angleController.setPID(p, i, d);
    }

}
