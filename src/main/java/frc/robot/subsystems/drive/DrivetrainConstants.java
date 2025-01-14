package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public final class DrivetrainConstants {
  public static final double TURN_CRUISE_VELOCITY = 100.0; // in rot/s
  public static final double TURN_ACCELERATION_MULTIPLIER = 10.0;
  public static final double TURN_FEEDFORWARD_kV = 0.12;
  public static final double TURN_FEEDFORWARD_kA = 0.1;

  public static final double maxLinearSpeed = 6.0;
  public static final double maxAngularSpeed = Math.PI * 2;
  public static final double joystickDeadband = 0.1;

  public final class PathPlannerConstants {
    public static final double ROBOT_MASS_KG = 74.088;
    public static final double ROBOT_MOI = 6.883;
    public static final double WHEEL_COF = 1.2;
  }

  public final class SimConstants {
    public static final GyroSimulation gyroSimulation = new GyroSimulation(0, 0);

    public static final DriveTrainSimulationConfig driveSimConfig =
        new DriveTrainSimulationConfig(
            Kilograms.of(DrivetrainConstants.PathPlannerConstants.ROBOT_MASS_KG),
            Meters.of(Drive.DRIVE_BASE_LENGTH + 0.05),
            Meters.of(Drive.DRIVE_BASE_WIDTH + 0.05),
            Meters.of(Drive.DRIVE_BASE_LENGTH),
            Meters.of(Drive.DRIVE_BASE_WIDTH),
            COTS.ofMark4(DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1), 1.2, 2),
            () -> DrivetrainConstants.SimConstants.gyroSimulation);
  }
}
