package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbIOSim implements ClimbIO {
    
    private SingleJointedArmSim climb = new SingleJointedArmSim(DCMotor.getKrakenX60(2),75, SingleJointedArmSim.estimateMOI(0.75, 0.5), 0.5, 0, 90, false, 0, null);

    private final PIDController angleController =
            new PIDController(0,0,0);

    private MutAngle goalAngle = Radians.mutable(0);
    private MutVoltage overrideVoltage = Volts.mutable(0.0);

    private boolean override = false;

    public ClimbIOSim() {
        SmartDashboard.putBoolean("lockedToCage", false);
    }
    
    @Override
    public void updateInputs(ClimbInputs inputs) {

        inputs.lockedToCage = SmartDashboard.getBoolean("lockedToCage", false);
        inputs.goalAngle.mut_replace(goalAngle);
        inputs.motorAngle.mut_replace(Radians.of(climb.getAngleRads()));
    }

    @Override
    public void applyOutputs(ClimbOutputs outputs) {
        Voltage appliedVolts;
        if (override) {
            appliedVolts = overrideVoltage;
        } else {
            appliedVolts = Volts.of(angleController.calculate(goalAngle.in(Radians), climb.getAngleRads()));
        }

        outputs.appliedVoltage.mut_replace(appliedVolts);
        climb.setInputVoltage(outputs.appliedVoltage.in(Volt));
    }

    @Override 
    public void setGoalAngle(Angle angle) {
        override = false;
        goalAngle.mut_replace(angle);
    }

    @Override
    public void setOverrideVoltage(Voltage voltage) {
        override = true;
        overrideVoltage.mut_replace(voltage);
    }

    @Override
    public void setPID(double p, double i, double d) {
        angleController.setPID(p, i, d);
    }

}
