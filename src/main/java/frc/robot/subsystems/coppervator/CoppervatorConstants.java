package frc.robot.subsystems.coppervator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.ElevatorMechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;
import frc.robot.constants.subsystems.ElevatorConstants;

public final class CoppervatorConstants {
   private CoppervatorConstants() {} 
   
   private static int encoderId = 52;

   public static MechanismConfig mechanismConfig = ElevatorMechanismConfig.builder()
        .withName("coppervator")
        .withEncoderToMechanismRatio(1.0)
        .withMotorToEncoderRatio(2.0)
        .withLeadMotorId(new CANDeviceID("canivore", 50))
        .withGravityFeedforwardType(GravityFeedforwardType.STATIC_ELEVATOR)
        .addFollower(new CANDeviceID("canivore", 51), true)
        .build();

    public TalonFXConfiguration talonFXConfig =
        new TalonFXConfiguration()
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(encoderId)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withSensorToMechanismRatio(1.0)
                    .withRotorToSensorRatio(
                        ElevatorConstants.synced.getObject().rotorToLargeCANCoderRatio))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(
                        ElevatorConstants.synced.getObject().elevatorStatorCurrentLimit))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKS(ElevatorConstants.synced.getObject().elevatorkS)
                    .withKV(ElevatorConstants.synced.getObject().elevatorkV)
                    .withKA(ElevatorConstants.synced.getObject().elevatorkA)
                    .withKG(ElevatorConstants.synced.getObject().elevatorkG)
                    .withKP(ElevatorConstants.synced.getObject().elevatorkP)
                    .withKI(ElevatorConstants.synced.getObject().elevatorkI)
                    .withKD(ElevatorConstants.synced.getObject().elevatorkD))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        ElevatorConstants.synced.getObject().elevatorAngularCruiseVelocity)
                    .withMotionMagicExpo_kA(
                        ElevatorConstants.synced.getObject().elevatorExpo_kA_raw)
                    .withMotionMagicExpo_kV(
                        ElevatorConstants.synced.getObject().elevatorExpo_kV_raw));
}
