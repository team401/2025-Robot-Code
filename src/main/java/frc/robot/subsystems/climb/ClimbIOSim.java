package frc.robot.subsystems.climb;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimbIOSim implements ClimbIO {
    
    private boolean lockedToCage;
    private TalonFXSimState simMotor;
    private CANcoderSimState simAngleCoder;

    private SingleJointedArmSim climb = new SingleJointedArmSim(DCMotor.getKrakenX60(2),75, SingleJointedArmSim.estimateMOI(0.75, 0.5), 0.5, 0, 90, false, 0, null);

    
    private final PIDController angleController =
            new PIDController(0,0,0);

            
    private final SimpleMotorFeedforward angleFeedforward =
            new SimpleMotorFeedforward(0,0,0);

    public ClimbIOSim() {
        simMotor = new TalonFXSimState(null);
        simAngleCoder = new CANcoderSimState(null);
    }

    @Override
    public void updateInputs(ClimbInputs inputs) {
        inputs.lockedToCage = this.lockedToCage;
        //inputs.motorVoltage.mut_replace(Volts.of());
        inputs.motorAngle.mut_replace(Radians.of(climb.getAngleRads()));
    }

    @Override
    public void setPID(double p, double i, double d) {
        angleController.setPID(p, i, d);
    }

    @Override
    public void setFF(double kS, double kV, double kA, double kG) {
        //angleFeedforward.
    }
}
