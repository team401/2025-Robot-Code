package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import coppercore.math.Deadband;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.UnitUtils;
import coppercore.wpilib_interface.tuning.Tunable;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.subsystems.ElevatorConstants;
import frc.robot.subsystems.scoring.ElevatorIO.ElevatorOutputMode;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorMechanism implements Tunable {
  ElevatorIO io;
  ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
  ElevatorOutputsAutoLogged outputs = new ElevatorOutputsAutoLogged();

  MutDistance goalHeight = Meters.mutable(0.0);
  MutDistance clampedGoalHeight = Meters.mutable(0.0);

  Distance minHeight = ElevatorConstants.synced.getObject().minElevatorHeight;
  Distance maxHeight = ElevatorConstants.synced.getObject().maxElevatorHeight;

  LoggedTunableNumber elevatorkP;
  LoggedTunableNumber elevatorkI;
  LoggedTunableNumber elevatorkD;

  LoggedTunableNumber elevatorkS;
  LoggedTunableNumber elevatorkV;
  LoggedTunableNumber elevatorkA;
  LoggedTunableNumber elevatorkG;

  LoggedTunableNumber elevatorExpokV;
  LoggedTunableNumber elevatorExpokA;

  LoggedTunableNumber elevatorTuningSetpointMeters;
  LoggedTunableNumber elevatorTuningOverrideAmps;

  // Has the elevator been seeded with CRT yet?
  // This exists in case we fail to seed with CRT the first try, it will try again each tick until
  // it succeeds.
  private boolean hasBeenSeeded = false;

  MedianFilter largeCANcoderFilter =
      new MedianFilter(JsonConstants.elevatorConstants.medianFilterWindowSize);
  MedianFilter smallCANcoderFilter =
      new MedianFilter(JsonConstants.elevatorConstants.medianFilterWindowSize);

  DoubleSupplier tuningHeightSetpointAdjustmentSupplier = () -> 0.0;

  public ElevatorMechanism(ElevatorIO io) {
    elevatorkP =
        new LoggedTunableNumber(
            "ElevatorTunables/elevatorkP", ElevatorConstants.synced.getObject().elevatorkP);
    elevatorkI =
        new LoggedTunableNumber(
            "ElevatorTunables/elevatorkI", ElevatorConstants.synced.getObject().elevatorkI);
    elevatorkD =
        new LoggedTunableNumber(
            "ElevatorTunables/elevatorkD", ElevatorConstants.synced.getObject().elevatorkD);

    elevatorkS =
        new LoggedTunableNumber(
            "ElevatorTunables/elevatorkS", ElevatorConstants.synced.getObject().elevatorkS);
    elevatorkV =
        new LoggedTunableNumber(
            "ElevatorTunables/elevatorkV", ElevatorConstants.synced.getObject().elevatorkV);
    elevatorkA =
        new LoggedTunableNumber(
            "ElevatorTunables/elevatorkA", ElevatorConstants.synced.getObject().elevatorkA);
    elevatorkG =
        new LoggedTunableNumber(
            "ElevatorTunables/elevatorkG", ElevatorConstants.synced.getObject().elevatorkG);

    elevatorExpokV =
        new LoggedTunableNumber(
            "ElevatorTunables/elevatorExpokV",
            ElevatorConstants.synced.getObject().elevatorExpo_kV.magnitude());
    elevatorExpokA =
        new LoggedTunableNumber(
            "ElevatorTunables/elevatorExpokA",
            ElevatorConstants.synced.getObject().elevatorExpo_kA.magnitude());

    elevatorTuningSetpointMeters =
        new LoggedTunableNumber("ElevatorTunables/elevatorTuningSetpointMeters", 0.0);
    elevatorTuningOverrideAmps =
        new LoggedTunableNumber("ElevatorTunables/elevatorTuningOverrideAmps", 0.0);

    this.io = io;

    periodic();

    // Initialize CRT logging fields with sentinel values so we can see them in AdvantageScope
    // before CRT is finished
    Logger.recordOutput("elevator/CRTSolutionSpoolAngle", Rotations.of(-1.0));
    Logger.recordOutput("elevator/CRTSolutionHeight", Meters.of(-1.0));
    Logger.recordOutput("elevator/seededWithCRT", false);

    // Seed elevator height using CRT on initialize
    seedWithCRT();
  }

  /**
   * Has the elevator position been seeded yet?
   *
   * @return True if it has been seeded, false if it hasn't been seeded.
   */
  public boolean hasBeenSeeded() {
    return hasBeenSeeded;
  }

  public void periodic() {
    Logger.recordOutput("elevator/hasBeenSeeded", hasBeenSeeded);
    if (inputs.largeEncoderConnected && inputs.smallEncoderConnected && !hasBeenSeeded) {
      if (!JsonConstants.elevatorConstants.ignoreCRT) {
        seedWithCRT();
      }
    }

    sendGoalHeightToIO();

    io.updateInputs(inputs);
    io.applyOutputs(outputs);

    Logger.recordOutput("elevator/height", getElevatorHeight());
    Logger.recordOutput("elevator/goalHeight", goalHeight);
    Logger.recordOutput("elevator/clampedGoalHeight", clampedGoalHeight);

    Logger.processInputs("elevator/inputs", inputs);
    Logger.processInputs("elevator/outputs", outputs);
  }

  /** This method must be called from the subsystem's test periodic! */
  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case ElevatorTuning:
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid) -> {
              io.setPID(pid[0], pid[1], pid[2]);
            },
            elevatorkP,
            elevatorkI,
            elevatorkD);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (ff) -> {
              io.setFF(ff[0], ff[1], ff[2], ff[3]);
            },
            elevatorkS,
            elevatorkV,
            elevatorkA,
            elevatorkG);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (maxProfile) -> {
              io.setMaxProfile(
                  RadiansPerSecond.of(0.0),
                  VoltsPerRadianPerSecondSquared.ofNative(maxProfile[0]),
                  VoltsPerRadianPerSecond.ofNative(maxProfile[1]));
            },
            elevatorExpokA,
            elevatorExpokV);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (current) -> {
              io.setOverrideCurrent(Amps.of(current[0]));
              io.setOutputMode(ElevatorOutputMode.Current);
            },
            elevatorTuningOverrideAmps);

      case SetpointTuning:
        // Allow setpointing the elevator in ElevatorTuning and SetpointTuning modes
        final double deadband = 0.17;

        double deadbandedJoystick =
            Deadband.oneAxisDeadband(
                tuningHeightSetpointAdjustmentSupplier.getAsDouble(), deadband);

        if (Math.abs(deadbandedJoystick) > deadband) {
          elevatorTuningSetpointMeters =
              new LoggedTunableNumber(
                  "ElevatorTunables/elevatorTuningSetpointMeters",
                  goalHeight.in(Meters) + deadbandedJoystick * 0.02);
        }

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (setpoint) -> {
              setGoalHeight(Meters.of(setpoint[0]));
              io.setOutputMode(ElevatorOutputMode.ClosedLoop);
            },
            elevatorTuningSetpointMeters);
        break;
      case WristClosedLoopTuning:
      case WristVoltageTuning:
        // When tuning the wrist, go to half a meter to avoid destroying outselves
        setGoalHeight(Meters.of(0.5));
        setOutputMode(ElevatorOutputMode.ClosedLoop);
      default:
        break;
    }
  }

  public void seedWithCRT() {
    final double filteredLargeEncoderAbsPos =
        largeCANcoderFilter.calculate(inputs.largeEncoderAbsolutePos.in(Rotations));
    final double filteredSmallEncoderAbsPos =
        smallCANcoderFilter.calculate(inputs.smallEncoderAbsolutePos.in(Rotations));

    Logger.recordOutput(
        "elevator/CRT/filteredLargeEncoderAbsPos",
        Units.rotationsToRadians(filteredLargeEncoderAbsPos));

    final int ticks = ElevatorConstants.synced.getObject().CRTticksPerRotation;
    final int smallTeeth = ElevatorConstants.synced.getObject().smallCANCoderTeeth;
    final int largeTeeth = ElevatorConstants.synced.getObject().largeCANCoderTeeth;
    final int spoolTeeth = ElevatorConstants.synced.getObject().spoolTeeth;
    // Find the number of ticks of each encoder, but in terms of the spool.
    // These should be multiplied by 19/18 or 17/18 (the gear ratios of the CANCoders to the
    // spool), but since the resulting numbers
    // aren't divisible by 18, this would result in rounding losing precision.
    // Therefore, we just multiply by 19 or 17 and then divide the final result by
    // 18.
    long ticksSmall = (long) Math.floor(filteredSmallEncoderAbsPos * ticks * smallTeeth);
    long ticksLarge = (long) Math.floor(filteredLargeEncoderAbsPos * ticks * largeTeeth);

    Logger.recordOutput("elevator/CRT/ticksSmall", ticksSmall);
    Logger.recordOutput("elevator/CRT/ticksLarge", ticksLarge);

    long solutionTicks = -1;

    for (int i = 0; i < largeTeeth; i++) {
      // // Try the offset of each multiple of 19 * ticks
      long potentialPosition = i * largeTeeth * ticks + ticksLarge;
      // Check whether that potential position is encoder 17's remainder away from a
      // multiple of 17
      if ((potentialPosition - ticksSmall) % (smallTeeth * ticks) == 0) {
        // If both conditions are met, we have a solution.
        solutionTicks = potentialPosition;
        break;
      }
    }

    Logger.recordOutput("elevator/CRT/solutionTicks", solutionTicks);

    if (solutionTicks != -1) {
      // Factor out the 18 from earlier.
      Angle solutionSpoolAngle =
          Rotations.of((double) solutionTicks / (double) ticks / (double) spoolTeeth);
      // The 19 tooth encoder will have turned 18/19 of a rotation for each rotation
      // of the spool
      Angle solutionLargeEncAngle =
          solutionSpoolAngle.times((double) spoolTeeth / (double) largeTeeth);
      // The 17 tooth encoder will have turned 18/17 of a rotation for each rotation
      // of the spool
      Angle solutionSmallEncAngle =
          solutionSpoolAngle.times((double) spoolTeeth / (double) smallTeeth);

      // Seed the encoder positions so that they are now accurate
      io.setLargeCANCoderPosition(solutionLargeEncAngle);
      io.setSmallCANCoderPosition(solutionSmallEncAngle);

      hasBeenSeeded = true;

      Logger.recordOutput("elevator/CRTSolutionSpoolAngle", solutionSpoolAngle);
      Logger.recordOutput(
          "elevator/CRTSolutionHeight",
          Inches.of(solutionSpoolAngle.in(Rotations) * 4.724).in(Meters));
      Logger.recordOutput("elevator/seededWithCRT", true);
    } else {
      System.out.println("ERROR: Couldn't find solution to seed elevator with CRT");
    }
  }

  /**
   * Seed the elevator to zero height
   *
   * <p>This should be called after a homing routine determines the elevator is at zero
   */
  public void seedToZero() {
    io.setLargeCANCoderPosition(Rotations.zero());
    io.setSmallCANCoderPosition(Rotations.zero());

    Logger.recordOutput("elevator/CRTSolutionSpoolAngle", Rotations.of(0.0));
    Logger.recordOutput("elevator/CRTSolutionHeight", Meters.of(0.0));
    Logger.recordOutput("elevator/seededWithCRT", false);

    hasBeenSeeded = true;
  }

  /**
   * Set the allowed range of motion for the elevator.
   *
   * <p>When not in override mode, the elevator will clamp its goal height to be within these
   * bounds. If the elevator is outside of these bounds, it will update its goal position and
   * control to be back within these bounds as soon as it can. This can be used to restrict the
   * allowed positions of the elevator, for instance to stop it from destroying a mechanism attached
   * to the elevator when that mechanism is in a certain position.
   */
  public void setAllowedRangeOfMotion(Distance minHeight, Distance maxHeight) {
    this.minHeight = minHeight;
    this.maxHeight = maxHeight;
  }

  public void setMinAllowedHeight(Distance minHeight) {
    this.minHeight = minHeight;
  }

  public void setMaxAllowedHeight(Distance maxHeight) {
    this.maxHeight = maxHeight;
  }

  /**
   * Set the goal height which the elevator will control to when it is not in override mode
   *
   * <p>This goal height will be clamped by the allowed range of motion set by
   * setAllowedRangeOfMotion before it is sent to the elevator io.
   */
  public void setGoalHeight(Distance goalHeight) {
    this.goalHeight.mut_replace(goalHeight);

    Logger.recordOutput("elevator/goalHeight", goalHeight);
  }

  /**
   * Based on the previously set goal height, update the clamped goal height to be within the
   * current bounds.
   */
  private void updateClampedGoalHeight() {
    Logger.recordOutput("elevator/minHeight", minHeight);
    Logger.recordOutput("elevator/maxHeight", maxHeight);
    clampedGoalHeight.mut_replace(UnitUtils.clampMeasure(goalHeight, minHeight, maxHeight));
  }

  /**
   * Clamp the goal height, then convert it to rotations of the large encoder and send the goal
   * rotations to the IO.
   */
  private void sendGoalHeightToIO() {
    updateClampedGoalHeight();

    // TODO: Use coppercore gear math after https://github.com/team401/coppercore/issues/52 is
    // done.

    Angle spoolRotations =
        Rotations.of(
            clampedGoalHeight
                .div(ElevatorConstants.synced.getObject().elevatorHeightPerSpoolRotation)
                .magnitude());
    Angle largeEncoderRotations =
        spoolRotations.times(
            (double) ElevatorConstants.synced.getObject().spoolTeeth
                / (double) ElevatorConstants.synced.getObject().largeCANCoderTeeth);

    Logger.recordOutput("elevator/sentGoalRotations", largeEncoderRotations);
    io.setLargeCANCoderGoalPos(largeEncoderRotations);
  }

  /**
   * Get the current height of the elevator
   *
   * <p>This value is calculated by the current position of the large CANcoder, converted using
   * necessary ratios back into rotations of the spool and then height of the elevator. This means
   * that this value is affected by any error in CRT seed and backlash causing error in large
   * CANcoder position.
   *
   * @return Inferred estimate of the elevator height
   */
  public Distance getElevatorHeight() {
    // Calculate spool rotations by: (largeEncoderPos * largeEncoderTeeth / spoolTeeth)
    Angle spoolAngle =
        inputs.largeEncoderPos.times(
            (double) ElevatorConstants.synced.getObject().largeCANCoderTeeth
                / (double) ElevatorConstants.synced.getObject().spoolTeeth);

    // TODO: Use coppercore gear math after https://github.com/team401/coppercore/issues/52 is
    // done.

    // Convert spool rotations to height by multiplying by height per rotation
    return Meters.of(
        spoolAngle.in(Rotations)
            * ElevatorConstants.synced.getObject().elevatorHeightPerSpoolRotation.in(Meters));
  }

  public LinearVelocity getElevatorVelocity() {
    AngularVelocity spoolVelocity =
        inputs.largeEncoderVel.times(
            (double) JsonConstants.elevatorConstants.largeCANCoderTeeth
                / (double) JsonConstants.elevatorConstants.spoolTeeth);

    return MetersPerSecond.of(
        spoolVelocity.in(RotationsPerSecond)
            * JsonConstants.elevatorConstants.elevatorHeightPerSpoolRotation.in(Meters));
  }

  /**
   * Get the current goal height of the elevator.
   *
   * <p>This is the unclamped goal height, and it will not be changed when clamps are updated.
   *
   * @return A Distance, the latest goal height set by setGoalHeight
   */
  public Distance getElevatorGoalHeight() {
    return goalHeight;
  }

  /**
   * Set whether the elevator should use closed-loop control, apply its override voltage, or apply
   * its override current.
   */
  public void setOutputMode(ElevatorOutputMode outputMode) {
    io.setOutputMode(outputMode);
  }

  /**
   * Set the voltage the elevator will apply when in Voltage override output mode
   *
   * @param volts The voltage to apply
   */
  public void setOverrideVoltage(Voltage volts) {
    io.setOverrideVoltage(volts);
  }

  /** Set the static current that will be applied when the elevator is in override mode. */
  public void setOverrideCurrent(Current current) {
    io.setOverrideCurrent(current);
  }

  /**
   * Get a reference to the elevator's IO. This should be used to update PID, motion profile, and
   * feed forward gains, and to set brake mode/disable motors. This method exists to avoid the need
   * to duplicate all of these functions between the mechanism and the IO.
   *
   * @return the elevator mechanism's IO
   */
  public ElevatorIO getIO() {
    return io;
  }

  /** Set whether or not the motors on the elevator should be disabled. */
  public void setMotorsDisabled(boolean disabled) {
    io.setMotorsDisabled(disabled);
  }

  /**
   * Set the supplier used to get the value of the joystick used to move the elevator setpoint in
   * setpoint tuning mode
   */
  public void setTuningHeightSetpointAdjustmentSupplier(DoubleSupplier newSupplier) {
    this.tuningHeightSetpointAdjustmentSupplier = newSupplier;
  }

  // ===== Tunable definitions: =====
  // As a 'tunable', elevator is in terms of Large CANcoder, since thats its remote sensor for
  // FusedCANcoder
  public Angle getPosition() {
    return inputs.largeEncoderPos;
  }

  public AngularVelocity getVelocity() {
    return inputs.elevatorMechanismVelocity;
  }

  public void setOutput(double output) {
    io.setOutputMode(ElevatorOutputMode.Current);
    io.setOverrideCurrent(Amps.of(output));
  }

  public void setPID(double p, double i, double d) {
    io.setPID(p, i, d);
  }

  public void setFF(double kS, double kV, double kA, double kG) {
    io.setFF(kS, kV, kA, kG);
  }

  public void setMaxProfileProperties(
      AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
    Per<VoltageUnit, AngularAccelerationUnit> kA = Volts.of(12.0).div(maxAcceleration);
    Per<VoltageUnit, AngularVelocityUnit> kV = Volts.of(12.0).div(maxVelocity);
    io.setMaxProfile(maxVelocity, kA, kV);
  }

  public void runToPosition(Angle position) {
    io.setLargeCANCoderGoalPos(position);
  }
}
