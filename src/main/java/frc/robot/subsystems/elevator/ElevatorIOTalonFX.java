package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    Distance elevatorGoalHeight;
    Voltage overrideVolts;
    boolean isOverriding;

    /* TODO: name this left/right or front/back
    I'm unsure of the physical configuration of these motors
    as of yet, so I'm giving them placeholder names
    */
    TalonFX leadMotor;
    TalonFX followerMotor;

    CANcoder canCoder19;
    CANcoder canCoder17;

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    public ElevatorIOTalonFX() {
        leadMotor = new TalonFX(ElevatorConstants.leadElevatorMotorId);
        followerMotor = new TalonFX(ElevatorConstants.followerElevatorMotorId);

        canCoder19 = new CANcoder(ElevatorConstants.elevatorCANCoder19ID);
        canCoder17 = new CANcoder(ElevatorConstants.elevatorCANCoder17ID);

        setStatorCurrentLimit(ElevatorConstants.elevatorStatorCurrentLimit);

        CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        cancoderConfiguration.MagnetSensor.SensorDirection = ElevatorConstants.elevatorCANCoder19Direction;
        canCoder19.getConfigurator().apply(cancoderConfiguration);

        cancoderConfiguration.MagnetSensor.SensorDirection = ElevatorConstants.elevatorCANCoder17Direction;
        canCoder17.getConfigurator().apply(cancoderConfiguration);

        talonFXConfigs.Feedback.FeedbackRemoteSensorID = canCoder17.getDeviceID();
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfigs.Feedback.SensorToMechanismRatio = ElevatorConstants.CANCoder17ToMechanismRatio;
        talonFXConfigs.Feedback.RotorToSensorRatio = ElevatorConstants.RotorToCANCoder17Ratio;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.elevatorStatorCurrentLimit.in(Amps);

        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        slot0Configs.kS = ElevatorConstants.elevatorkS;
        slot0Configs.kV = ElevatorConstants.elevatorkV;
        slot0Configs.kA = ElevatorConstants.elevatorkA;
        slot0Configs.kG = ElevatorConstants.elevatorkG;

        slot0Configs.kP = ElevatorConstants.elevatorkP;
        slot0Configs.kI = ElevatorConstants.elevatorkI;
        slot0Configs.kD = ElevatorConstants.elevatorkD;

        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.elevatorCruiseVelocity.in(MetersPerSecond);
        motionMagicConfigs.MotionMagicExpo_kA = ElevatorConstants.elevatorExpo_kA.in(VoltsPerMeterPerSecondSquared);
        motionMagicConfigs.MotionMagicExpo_kV = ElevatorConstants.elevatorExpo_kV.in(VoltsPerMeterPerSecond);

        leadMotor.getConfigurator().apply(talonFXConfigs);

        followerMotor.setControl(new Follower(leadMotor.getDeviceID(), ElevatorConstants.invertFollowerElevatorMotor));
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
    }    

    @Override
    public void applyOutputs(ElevatorOutputs outputs) {
    }

    public void seedWithCRT() {}

    public void setGoalHeight(Distance goalHeight) {}

    public void setOverrideVolts(Voltage volts) {
        overrideVolts = volts;
    }

    public void setOverrideMode(boolean override) {
        isOverriding = override;
    }

    public void setPID(double p, double i, double d) {}

    public void setMaxProfile(LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {}

    public void setFF(double kS, double kV, double kA, double kG) {}

    public void setBrakeMode(boolean brakeMode) {}

    /** Set the stator current limit for both elevator motors */
    public void setStatorCurrentLimit(Current currentLimit) {}

    public void setMotorsDisabled(boolean disabled) {}
}
