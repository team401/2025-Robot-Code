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

    TalonFXConfiguration talonFXConfigs;

    // Has the elevator been seeded with CRT yet?
    // This exists in case we fail to seed with CRT the first try, it will try again each tick until it succeeds.
    boolean hasBeenSeeded = false;

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
        if (!hasBeenSeeded) {
            seedWithCRT();
        }
    }    

    @Override
    public void applyOutputs(ElevatorOutputs outputs) {
    }

    public void seedWithCRT() {
        final int ticks = ElevatorConstants.CRTticksPerRotation;
        // Find the number of ticks of each encoder, but in terms of the spool.
        // These should be multiplied by 19/18 or 17/18, but since the resulting numbers aren't divisible by 18, this would result in rounding losing precision.
        // Therefore, we just multiply by 19 or 17 and then divide the final result by 18.
        long ticks17 = Math.round(canCoder17.getAbsolutePosition().getValue().in(Rotations) * ticks * 19.0);
        long ticks19 = Math.round(canCoder19.getAbsolutePosition().getValue().in(Rotations) * ticks * 17.0);

        long solutionTicks = -1;

        for (int i = 0; i < 17; i++) {
            // Try the offset of each multiple of 19 * ticks
            long potentialPosition = i * 19 * ticks + ticks19;
            // Check whether that potential position is encoder 17's remainder away from a multiple of 17
            if ((potentialPosition - ticks17) % (17 * ticks) == 0) {
                // If both conditions are met, we have a solution.
                solutionTicks = potentialPosition;
                break;
            }
        }

        if (solutionTicks != -1) {
            // Factor out the 18 from earlier.
            Angle solutionSpoolAngle = Rotations.of((double) solutionTicks / (double) ticks / 18.0);
            // The 19 tooth encoder will have turned 18/19 of a rotation for each rotation of the spool
            Angle solutionEnc19Angle = solutionSpoolAngle.times(18.0/19.0);
            // The 17 tooth encoder will have turned 18/17 of a rotation for each rotation of the spool
            Angle solutionEnc17Angle = solutionSpoolAngle.times(18.0/17.0);

            // Seed the encoder positions so that they are now accurate
            canCoder19.setPosition(solutionEnc19Angle);
            canCoder17.setPosition(solutionEnc17Angle);

            hasBeenSeeded = true;
        } else {
            System.out.println("ERROR: Couldn't find solution to seed elevator with CRT");
        }
        
    }

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
