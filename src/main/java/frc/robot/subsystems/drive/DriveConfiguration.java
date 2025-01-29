package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import frc.robot.constants.JsonConstants;

public class DriveConfiguration {
  private static DriveConfiguration instance = null;

  public Slot0Configs steerGains;
  public Slot0Configs driveGains;

  public SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator;

  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft;
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight;
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft;
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight;

  public DriveConfiguration() {
    steerGains =
        new Slot0Configs()
            .withKP(JsonConstants.drivetrainConstants.steerKp)
            .withKI(JsonConstants.drivetrainConstants.steerKi)
            .withKD(JsonConstants.drivetrainConstants.steerKd)
            .withKS(JsonConstants.drivetrainConstants.steerKs)
            .withKV(JsonConstants.drivetrainConstants.steerKv)
            .withKA(JsonConstants.drivetrainConstants.steerKa)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    driveGains =
        new Slot0Configs()
            .withKP(JsonConstants.drivetrainConstants.driveKp)
            .withKI(JsonConstants.drivetrainConstants.driveKi)
            .withKD(JsonConstants.drivetrainConstants.driveKd)
            .withKS(JsonConstants.drivetrainConstants.driveKs)
            .withKV(JsonConstants.drivetrainConstants.driveKv)
            .withKA(JsonConstants.drivetrainConstants.driveKa)
            .withKG(JsonConstants.drivetrainConstants.driveKg);

    ConstantCreator =
        new SwerveModuleConstantsFactory<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(JsonConstants.drivetrainConstants.kDriveGearRatio)
            .withSteerMotorGearRatio(JsonConstants.drivetrainConstants.kSteerGearRatio)
            .withCouplingGearRatio(JsonConstants.drivetrainConstants.kCoupleRatio)
            .withWheelRadius(JsonConstants.drivetrainConstants.kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(
                JsonConstants.drivetrainConstants.kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(
                JsonConstants.drivetrainConstants.kDriveClosedLoopOutput)
            .withSlipCurrent(JsonConstants.drivetrainConstants.kSlipCurrent)
            .withSpeedAt12Volts(JsonConstants.drivetrainConstants.kSpeedAt12Volts)
            .withDriveMotorType(JsonConstants.drivetrainConstants.kDriveMotorType)
            .withSteerMotorType(JsonConstants.drivetrainConstants.kSteerMotorType)
            .withFeedbackSource(JsonConstants.drivetrainConstants.kSteerFeedbackType)
            .withDriveMotorInitialConfigs(JsonConstants.drivetrainConstants.driveInitialConfigs)
            .withSteerMotorInitialConfigs(JsonConstants.drivetrainConstants.steerInitialConfigs)
            .withEncoderInitialConfigs(JsonConstants.drivetrainConstants.encoderInitialConfigs)
            .withSteerInertia(JsonConstants.drivetrainConstants.kSteerInertia)
            .withDriveInertia(JsonConstants.drivetrainConstants.kDriveInertia)
            .withSteerFrictionVoltage(JsonConstants.drivetrainConstants.kSteerFrictionVoltage)
            .withDriveFrictionVoltage(JsonConstants.drivetrainConstants.kDriveFrictionVoltage);

    FrontLeft =
        ConstantCreator.createModuleConstants(
            JsonConstants.drivetrainConstants.kFrontLeftSteerMotorId,
            JsonConstants.drivetrainConstants.kFrontLeftDriveMotorId,
            JsonConstants.drivetrainConstants.kFrontLeftEncoderId,
            JsonConstants.drivetrainConstants.kFrontLeftEncoderOffset,
            JsonConstants.drivetrainConstants.kFrontLeftXPos,
            JsonConstants.drivetrainConstants.kFrontLeftYPos,
            JsonConstants.drivetrainConstants.kInvertLeftSide,
            JsonConstants.drivetrainConstants.kFrontLeftSteerMotorInverted,
            JsonConstants.drivetrainConstants.kFrontLeftEncoderInverted);

    FrontRight =
        ConstantCreator.createModuleConstants(
            JsonConstants.drivetrainConstants.kFrontRightSteerMotorId,
            JsonConstants.drivetrainConstants.kFrontRightDriveMotorId,
            JsonConstants.drivetrainConstants.kFrontRightEncoderId,
            JsonConstants.drivetrainConstants.kFrontRightEncoderOffset,
            JsonConstants.drivetrainConstants.kFrontRightXPos,
            JsonConstants.drivetrainConstants.kFrontRightYPos,
            JsonConstants.drivetrainConstants.kInvertRightSide,
            JsonConstants.drivetrainConstants.kFrontRightSteerMotorInverted,
            JsonConstants.drivetrainConstants.kFrontRightEncoderInverted);

    BackLeft =
        ConstantCreator.createModuleConstants(
            JsonConstants.drivetrainConstants.kBackLeftSteerMotorId,
            JsonConstants.drivetrainConstants.kBackLeftDriveMotorId,
            JsonConstants.drivetrainConstants.kBackLeftEncoderId,
            JsonConstants.drivetrainConstants.kBackLeftEncoderOffset,
            JsonConstants.drivetrainConstants.kBackLeftXPos,
            JsonConstants.drivetrainConstants.kBackLeftYPos,
            JsonConstants.drivetrainConstants.kInvertLeftSide,
            JsonConstants.drivetrainConstants.kBackLeftSteerMotorInverted,
            JsonConstants.drivetrainConstants.kBackLeftEncoderInverted);

    BackRight =
        ConstantCreator.createModuleConstants(
            JsonConstants.drivetrainConstants.kBackRightSteerMotorId,
            JsonConstants.drivetrainConstants.kBackRightDriveMotorId,
            JsonConstants.drivetrainConstants.kBackRightEncoderId,
            JsonConstants.drivetrainConstants.kBackRightEncoderOffset,
            JsonConstants.drivetrainConstants.kBackRightXPos,
            JsonConstants.drivetrainConstants.kBackRightYPos,
            JsonConstants.drivetrainConstants.kInvertRightSide,
            JsonConstants.drivetrainConstants.kBackRightSteerMotorInverted,
            JsonConstants.drivetrainConstants.kBackRightEncoderInverted);
  }

  public static DriveConfiguration getInstance() {
    if (instance == null) {
      instance = new DriveConfiguration();
    }

    return instance;
  }
}
