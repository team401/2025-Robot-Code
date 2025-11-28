package frc.robot.subsystems.coppervator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import coppercore.wpilib_interface.MonitoredSubsystem;
import coppercore.wpilib_interface.subsystems.encoders.EncoderIO;
import coppercore.wpilib_interface.subsystems.encoders.EncoderInputsAutoLogged;
import coppercore.wpilib_interface.subsystems.motors.MotorIO;
import coppercore.wpilib_interface.subsystems.motors.MotorInputsAutoLogged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.subsystems.ElevatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Coppervator is a subsystem written to demonstrate coppercore's IO classes by implementing an
 * elevator.
 */
public class CoppervatorSubsystem extends MonitoredSubsystem {
  MotorInputsAutoLogged leadMotorInputs = new MotorInputsAutoLogged();
  MotorInputsAutoLogged followerMotorInputs = new MotorInputsAutoLogged();

  MotorIO leadMotor;
  MotorIO followerMotor;

  EncoderInputsAutoLogged cancoderInputs = new EncoderInputsAutoLogged();
  EncoderIO encoder;

  // Use this variable as test, just "stair-step" it whenever we reach goal angle.
  @AutoLogOutput(key = "Coppervator/goalHeight")
  double currentGoalHeightMeters = 0.0;

  final double heightIncrement = 0.75;

  public CoppervatorSubsystem(MotorIO leaderIO, MotorIO followerIO, EncoderIO cancoderIO) {
    this.leadMotor = leaderIO;
    this.followerMotor = followerIO;
    this.encoder = cancoderIO;
  }

  @Override
  public void monitoredPeriodic() {
    // "subsystem" duties
    if (DriverStation.isEnabled()
        && getHeight().isNear(Meters.of(currentGoalHeightMeters), Meters.of(0.03))) {
      currentGoalHeightMeters += 0.75;
      currentGoalHeightMeters %= JsonConstants.elevatorConstants.maxElevatorHeight.in(Meters);
      if (currentGoalHeightMeters < JsonConstants.elevatorConstants.minElevatorHeight.in(Meters)) {
        currentGoalHeightMeters = JsonConstants.elevatorConstants.minElevatorHeight.in(Meters);
      }
    }

    setGoalHeight(Meters.of(currentGoalHeightMeters));
    // "mechanism" duties
    leadMotor.updateInputs(leadMotorInputs);
    followerMotor.updateInputs(followerMotorInputs);

    encoder.updateInputs(cancoderInputs);

    Logger.processInputs("coppervator/leadMotorInputs", leadMotorInputs);
    Logger.processInputs("coppervator/followerMotorInputs", followerMotorInputs);
    Logger.processInputs("coppervator/encoderInputs", cancoderInputs);
  }

  public static ElevatorSim createElevatorSim() {
    return new ElevatorSim(
        DCMotor.getKrakenX60Foc(2),
        ElevatorConstants.synced.getObject().elevatorReduction,
        ElevatorConstants.synced.getObject().carriageMass.in(Kilograms),
        ElevatorConstants.synced.getObject().drumRadius.in(Meters),
        ElevatorConstants.synced.getObject().minElevatorHeight.in(Meters),
        ElevatorConstants.synced.getObject().maxElevatorHeight.in(Meters),
        true,
        ElevatorConstants.Sim.synced.getObject().elevatorStartingHeight.in(Meters),
        ElevatorConstants.Sim.synced.getObject().positionStdDev,
        ElevatorConstants.Sim.synced.getObject().velocityStdDev);
  }

  private void setGoalHeight(Distance goalHeight) {
    var config = CoppervatorConstants.mechanismConfig;
    Measure<AngleUnit> spoolRotationsMeasure =
        goalHeight.timesRatio(config.elevatorToMechanismRatio.reciprocal());
    Angle spoolRotationsAngle =
        Angle.ofBaseUnits(
            spoolRotationsMeasure.baseUnitMagnitude(), spoolRotationsMeasure.baseUnit());

    leadMotor.controlToPositionExpoProfiled(spoolRotationsAngle);
  }

  @AutoLogOutput(key = "Coppervator/height")
  public Distance getHeight() {
    var config = CoppervatorConstants.mechanismConfig;
    Angle encoderPos = Radians.of(cancoderInputs.positionRadians);
    Angle spoolPos = encoderPos.div(config.encoderToMechanismRatio);

    Measure<DistanceUnit> heightMeasure = spoolPos.timesRatio(config.elevatorToMechanismRatio);

    Distance height = heightMeasure.unit().of(heightMeasure.magnitude());
    return height;
  }
}
