package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    MutDistance elevatorGoalHeight;
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

    TalonFXConfiguration talonFXConfigs;

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

        talonFXConfigs = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(canCoder19.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(ElevatorConstants.CANCoder19ToMechanismRatio)
                .withRotorToSensorRatio(ElevatorConstants.RotorToCANCoder19Ratio))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(ElevatorConstants.elevatorStatorCurrentLimit))
            .withSlot0(new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKS(ElevatorConstants.elevatorkS)
                .withKV(ElevatorConstants.elevatorkV)
                .withKA(ElevatorConstants.elevatorkA)
                .withKG(ElevatorConstants.elevatorkG)

                .withKP(ElevatorConstants.elevatorkP)
                .withKI(ElevatorConstants.elevatorkI)
                .withKD(ElevatorConstants.elevatorkD))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(ElevatorConstants.elevatorAngularCruiseVelocity)
                .withMotionMagicExpo_kA(ElevatorConstants.elevatorExpo_kA)
                .withMotionMagicExpo_kV(ElevatorConstants.elevatorExpo_kV));

        leadMotor.getConfigurator().apply(talonFXConfigs);
        followerMotor.getConfigurator().apply(talonFXConfigs);

        followerMotor.setControl(new Follower(leadMotor.getDeviceID(), ElevatorConstants.invertFollowerElevatorMotor));
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.encoder19Pos.mut_replace(canCoder19.getPosition().getValue());
        inputs.encoder17Pos.mut_replace(canCoder17.getPosition().getValue());

        inputs.encoder19AbsolutePos.mut_replace(canCoder19.getAbsolutePosition().getValue());
        inputs.encoder17AbsolutePos.mut_replace(canCoder17.getAbsolutePosition().getValue());

        inputs.elevatorGoalHeight.mut_replace(elevatorGoalHeight);

        inputs.elevatorLeadMotorStatorCurrent.mut_replace(leadMotor.getStatorCurrent().getValue());
        inputs.elevatorLeadMotorSupplyCurrent.mut_replace(leadMotor.getSupplyCurrent().getValue());

        inputs.elevatorFollowerMotorStatorCurrent.mut_replace(followerMotor.getStatorCurrent().getValue());
        inputs.elevatorFollowerMotorSupplyCurrent.mut_replace(followerMotor.getSupplyCurrent().getValue());
    }

    @Override
    public void applyOutputs(ElevatorOutputs outputs) {
        // TODO: Control to a position
    }


    public void setGoalHeight(Distance goalHeight) {}

    public Angle getCANCoder19AbsPos() { return canCoder19.getAbsolutePosition().getValue(); }

    public Angle getCANCoder17AbsPos() { return canCoder17.getAbsolutePosition().getValue(); }

    public void setCANCoder19Position(Angle newAngle) {
        canCoder19.setPosition(newAngle);
    }

    public void setCANCoder17Position(Angle newAngle) {
        canCoder17.setPosition(newAngle);
    }

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
