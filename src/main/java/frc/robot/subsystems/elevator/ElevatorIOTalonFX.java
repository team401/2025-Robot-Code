package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    MutAngle largeEncoderGoalAngle;
    MutAngle largeEncoderSetpointPosition = Rotations.mutable(0.0);

    Voltage overrideVolts;
    boolean isOverriding;

    /* TODO: name this left/right or front/back
    I'm unsure of the physical configuration of these motors
    as of yet, so I'm giving them placeholder names
    */
    TalonFX leadMotor;
    TalonFX followerMotor;

    CANcoder largeCANCoder;
    CANcoder smallCANCoder;

    TalonFXConfiguration talonFXConfigs;

    boolean motorDisabled = false;

    // Reuse the same motion magic request to avoid garbage collector having to clean them up.
    MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0.0);
    VoltageOut voltageOut = new VoltageOut(0.0);

    public ElevatorIOTalonFX() {
        leadMotor = new TalonFX(ElevatorConstants.leadElevatorMotorId);
        followerMotor = new TalonFX(ElevatorConstants.followerElevatorMotorId);

        largeCANCoder = new CANcoder(ElevatorConstants.elevatorLargeCANCoderID);
        smallCANCoder = new CANcoder(ElevatorConstants.elevatorSmallCANCoderID);

        setStatorCurrentLimit(ElevatorConstants.elevatorStatorCurrentLimit);

        CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
        cancoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        cancoderConfiguration.MagnetSensor.SensorDirection = ElevatorConstants.elevatorLargeCANCoderDirection;
        largeCANCoder.getConfigurator().apply(cancoderConfiguration);

        cancoderConfiguration.MagnetSensor.SensorDirection = ElevatorConstants.elevatorSmallCANCoderDirection;
        smallCANCoder.getConfigurator().apply(cancoderConfiguration);

        talonFXConfigs = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(largeCANCoder.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(ElevatorConstants.largeCANCoderToMechanismRatio)
                .withRotorToSensorRatio(ElevatorConstants.rotorToLargeCANCoderRatio))
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
        inputs.largeEncoderPos.mut_replace(largeCANCoder.getPosition().getValue());
        inputs.smallEncoderPos.mut_replace(smallCANCoder.getPosition().getValue());

        inputs.largeEncoderAbsolutePos.mut_replace(largeCANCoder.getAbsolutePosition().getValue());
        inputs.smallEncoderAbsolutePos.mut_replace(smallCANCoder.getAbsolutePosition().getValue());

        inputs.largeEncoderGoalPos.mut_replace(largeEncoderGoalAngle);
        inputs.largeEncoderSetpointPos.mut_replace(largeEncoderSetpointPosition);

        inputs.elevatorLeadMotorStatorCurrent.mut_replace(leadMotor.getStatorCurrent().getValue());
        inputs.elevatorLeadMotorSupplyCurrent.mut_replace(leadMotor.getSupplyCurrent().getValue());

        inputs.elevatorFollowerMotorStatorCurrent.mut_replace(followerMotor.getStatorCurrent().getValue());
        inputs.elevatorFollowerMotorSupplyCurrent.mut_replace(followerMotor.getSupplyCurrent().getValue());
    }

    @Override
    public void applyOutputs(ElevatorOutputs outputs) {
        motionMagicExpoTorqueCurrentFOC.withPosition(largeEncoderGoalAngle);

        if (motorDisabled) {
            leadMotor.setControl(voltageOut.withOutput(0.0));
        } else if (isOverriding) {
            leadMotor.setControl(voltageOut.withOutput(overrideVolts));
        } else {
            leadMotor.setControl(motionMagicExpoTorqueCurrentFOC);
            largeEncoderSetpointPosition.mut_setMagnitude((leadMotor.getClosedLoopReference().getValue()));
        }
    }

    @Override
    public Angle getLargeCANCoderAbsPos() { return largeCANCoder.getAbsolutePosition().getValue(); }

    @Override
    public Angle getSmallCANCoderAbsPos() { return smallCANCoder.getAbsolutePosition().getValue(); }


    @Override
    public void setLargeCANCoderPosition(Angle newAngle) {
        largeCANCoder.setPosition(newAngle);
    }

    @Override
    public void setSmallCANCoderPosition(Angle newAngle) {
        smallCANCoder.setPosition(newAngle);
    }

    @Override
    public void setOverrideVolts(Voltage volts) {
        overrideVolts = volts;
    }

    @Override
    public void setOverrideMode(boolean override) {
        isOverriding = override;
    }

    @Override
    public void setPID(double p, double i, double d) {
        Slot0Configs configs = talonFXConfigs.Slot0;

        configs.kP = p;
        configs.kI = i;
        configs.kD = d;

        leadMotor.getConfigurator().apply(configs);
        followerMotor.getConfigurator().apply(configs);
    }

    @Override
    public void setMaxProfile(AngularVelocity maxVelocity, Per<VoltageUnit, AngularAccelerationUnit> expo_kA, Per<VoltageUnit, AngularVelocityUnit> expo_kV) {
        // TODO: Figure out how to handle maximum velocity for the elevator
        MotionMagicConfigs configs = talonFXConfigs.MotionMagic
            .withMotionMagicCruiseVelocity(maxVelocity)
            .withMotionMagicExpo_kA(expo_kA)
            .withMotionMagicExpo_kV(expo_kV);

        leadMotor.getConfigurator().apply(configs);
        followerMotor.getConfigurator().apply(configs);
    }

    @Override
    public void setFF(double kS, double kV, double kA, double kG) {
        Slot0Configs configs = talonFXConfigs.Slot0;

        configs.kS = kS;
        configs.kV = kV;
        configs.kA = kA;
        configs.kG = kG;

        leadMotor.getConfigurator().apply(configs);
        followerMotor.getConfigurator().apply(configs);
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        leadMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        followerMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setStatorCurrentLimit(Current currentLimit) {
        talonFXConfigs.CurrentLimits.withStatorCurrentLimit(currentLimit);

        // Only apply current limit configs to avoid overwriting PID and FF values from tuning
        leadMotor.getConfigurator().apply(talonFXConfigs.CurrentLimits);
        followerMotor.getConfigurator().apply(talonFXConfigs.CurrentLimits);
    }

    @Override
    public void setMotorsDisabled(boolean disabled) {
        motorDisabled = disabled;
    }
}
