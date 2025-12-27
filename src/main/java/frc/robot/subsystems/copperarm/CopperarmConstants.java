package frc.robot.subsystems.copperarm;

import com.revrobotics.spark.config.SparkMaxConfig;
import coppercore.wpilib_interface.subsystems.configs.CANDeviceID;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig;
import coppercore.wpilib_interface.subsystems.configs.MechanismConfig.GravityFeedforwardType;

public class CopperarmConstants {
  private CopperarmConstants() {}

  private static String canbus = "canivore";

  public static MechanismConfig mechanismConfig =
      MechanismConfig.builder()
          .withName("copperarm")
          .withEncoderToMechanismRatio(1.0)
          .withMotorToEncoderRatio(1.0)
          .withLeadMotorId(new CANDeviceID(canbus, 55))
          .withGravityFeedforwardType(GravityFeedforwardType.COSINE_ARM)
          .addFollower(new CANDeviceID(canbus, 56), true)
          .build();

  public static SparkMaxConfig getSparkMaxConfig() {
    var config = new SparkMaxConfig();

    config.closedLoop.p(10.0);
    config.closedLoop.i(0.0);
    config.closedLoop.d(0.0);

    return config;
  }
}
