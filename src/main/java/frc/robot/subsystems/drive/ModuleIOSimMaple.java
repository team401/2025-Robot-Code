package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSimMaple implements ModuleIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT =
      0.91035; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;

  // TODO:FIX
  private static final double DriveMotorGearRatio = 1.5;
  private static final double SteerMotorGearRatio = 1.5;
  private static final double WheelRadius = 0.05;

  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveSim;
  private final SimulatedMotorController.GenericMotorController turnSim;

  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSimMaple(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    // Create drive and turn sim

    // TODO: FIX
    SwerveModuleSimulationConfig configs =
        new SwerveModuleSimulationConfig(
            DRIVE_GEARBOX,
            TURN_GEARBOX,
            DriveMotorGearRatio,
            SteerMotorGearRatio,
            Voltage.ofBaseUnits(DRIVE_KS, Volts),
            Voltage.ofBaseUnits(DRIVE_KV_ROT, Volts),
            Distance.ofBaseUnits(WheelRadius, Meter),
            null,
            0);

    moduleSimulation = new SwerveModuleSimulation(configs);
    driveSim =
        moduleSimulation
            .useDriveMotorController(
                new SimulatedMotorController.GenericMotorController(DRIVE_GEARBOX))
            .withCurrentLimit(Current.ofRelativeUnits(60, Amp));

    turnSim =
        moduleSimulation
            .useSteerMotorController(
                new SimulatedMotorController.GenericMotorController(TURN_GEARBOX))
            // .useGenericControllerForSteer()
            .withCurrentLimit(Current.ofRelativeUnits(20, Amp));

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(
                  moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts =
          turnController.calculate(
              moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond));
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.requestVoltage(
        Voltage.ofRelativeUnits((MathUtil.clamp(driveAppliedVolts, -12.0, 12.0)), Volts));
    turnSim.requestVoltage(
        Voltage.ofRelativeUnits((MathUtil.clamp(turnAppliedVolts, -12.0, 12.0)), Volts));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad =
        moduleSimulation
            .getDriveWheelFinalPosition()
            .in(Radian); // driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amp));

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnPosition =
        new Rotation2d(moduleSimulation.getSteerRelativeEncoderPosition().in(Radian));
    inputs.turnVelocityRadPerSec =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amp));

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
