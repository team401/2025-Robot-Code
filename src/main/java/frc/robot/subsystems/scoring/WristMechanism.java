package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.UnitUtils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism to keep track of the Wrist
 *
 * <ul>
 *   <li>Uses closed-loop control, using torque-current FOC
 *   <li>Tracks its own position (0 is when the center of mass is at the same height as the joint,
 *       or 90 degrees up from where the wrist naturally hangs).
 */
public class WristMechanism {
  WristIO io;
  WristInputsAutoLogged inputs = new WristInputsAutoLogged();
  WristOutputsAutoLogged outputs = new WristOutputsAutoLogged();

  MutAngle goalAngle = Rotations.mutable(0.0);
  MutAngle clampedGoalAngle = Rotations.mutable(0.0);

  MutAngle minAngle = JsonConstants.wristConstants.wristMinMinAngle.mutableCopy();
  MutAngle maxAngle = JsonConstants.wristConstants.wristMaxMaxAngle.mutableCopy();

  LoggedTunableNumber wristkP;
  LoggedTunableNumber wristkI;
  LoggedTunableNumber wristkD;

  LoggedTunableNumber wristkS;
  LoggedTunableNumber wristkV;
  LoggedTunableNumber wristkA;
  LoggedTunableNumber wristkG;

  LoggedTunableNumber wristCruiseVelocity;
  LoggedTunableNumber wristExpokV;
  LoggedTunableNumber wristExpokA;

  LoggedTunableNumber wristTuningSetpointRotations;
  LoggedTunableNumber wristTuningOverrideAmps;

  public WristMechanism(WristIO io) {
    wristkP =
        new LoggedTunableNumber("WristTunables/wristkP", JsonConstants.wristConstants.wristKP);
    wristkI =
        new LoggedTunableNumber("WristTunables/wristkI", JsonConstants.wristConstants.wristKI);
    wristkD =
        new LoggedTunableNumber("WristTunables/wristkD", JsonConstants.wristConstants.wristKD);

    wristkS =
        new LoggedTunableNumber("WristTunables/wristkS", JsonConstants.wristConstants.wristKS);
    wristkV =
        new LoggedTunableNumber("WristTunables/wristkV", JsonConstants.wristConstants.wristKV);
    wristkA =
        new LoggedTunableNumber("WristTunables/wristkA", JsonConstants.wristConstants.wristKA);
    wristkG =
        new LoggedTunableNumber("WristTunables/wristkG", JsonConstants.wristConstants.wristKG);

    wristCruiseVelocity =
        new LoggedTunableNumber(
            "WristTunables/wristCruiseVelocity",
            JsonConstants.wristConstants.wristMotionMagicCruiseVelocity.in(RotationsPerSecond));
    wristExpokV =
        new LoggedTunableNumber(
            "WristTunables/wristExpokV", JsonConstants.wristConstants.wristMotionMagicExpo_kV);
    wristExpokA =
        new LoggedTunableNumber(
            "WristTunables/wristExpokA", JsonConstants.wristConstants.wristMotionMagicExpo_kA);

    wristTuningSetpointRotations =
        new LoggedTunableNumber("WristTunables/wristTuningSetpointRotations", 0.0);
    wristTuningOverrideAmps = new LoggedTunableNumber("WristTunables/wristTuningOverrideAmps", 0.0);

    this.io = io;
  }

  /**
   * Runs periodically when the robot is enabled
   *
   * <p>Does NOT run automatically! Must be called by the subsystem
   */
  public void periodic() {
    sendGoalAngleToIO();

    io.updateInputs(inputs);
    io.applyOutputs(outputs);

    Logger.processInputs("wrist/inputs", inputs);
    Logger.processInputs("wrist/outputs", outputs);
  }

  /** This method must be called from the subsystem's test periodic! */
  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case WristClosedLoopTuning:
        io.setOverrideMode(false);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid) -> {
              io.setPID(pid[0], pid[1], pid[2]);
            },
            wristkP,
            wristkI,
            wristkD);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (ff) -> {
              io.setFF(ff[0], ff[1], ff[2], ff[3]);
            },
            wristkS,
            wristkV,
            wristkA,
            wristkG);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (maxProfile) -> {
              io.setMaxProfile(
                  RadiansPerSecond.of(0.0),
                  VoltsPerRadianPerSecondSquared.ofNative(maxProfile[0]),
                  VoltsPerRadianPerSecond.ofNative(maxProfile[1]));
            },
            wristExpokA,
            wristExpokV);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (setpoint) -> {
              setGoalAngle(Rotations.of(setpoint[0]));
            },
            wristTuningSetpointRotations);
        break;
      case WristCurrentTuning:
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (setpoint) -> {
              io.setOverrideCurrent(Amps.of(setpoint[0]));
            },
            wristTuningOverrideAmps);
        io.setOverrideMode(true);
        break;
      default:
        break;
    }
  }

  public void sendGoalAngleToIO() {
    updateClampedGoalAngle();

    io.setWristGoalPos(clampedGoalAngle);
  }

  /**
   * Based on the bounds previously set, clamp the last set goal height to be between the bounds.
   *
   * <p>If the goal height is outside of the bounds and the bounds are expanded, this function will
   * still behave as expected, as the mechanism remembers its unclamped goal height and will attempt
   * to get there once it is allowed.
   */
  private void updateClampedGoalAngle() {
    clampedGoalAngle.mut_replace(UnitUtils.clampMeasure(goalAngle, minAngle, maxAngle));

    Logger.recordOutput("wrist/clampedGoalAngle", clampedGoalAngle);
  }

  /**
   * Set the goal angle the wrist will to control about.
   *
   * <p>This goal angle will be clamped by the allowed range of motion
   *
   * @param goalAngle The new goal angle
   */
  public void setGoalAngle(Angle goalAngle) {
    this.goalAngle.mut_replace(goalAngle);

    Logger.recordOutput("wrist/goalAngle", goalAngle);
  }

  /**
   * Sets the minimum and maximum allowed angles that the wrist may target.
   *
   * <p>When not in override mode, the goal angle of the wrist will be clamped to be between these
   * values before it is sent to the IO. When these clamps change, the original goal angle is
   * clamped to be within the new bounds.
   *
   * @param minAngle The minimum angle, which will be clamped between wristMinMinAngle and
   *     wristMaxMaxAngle before being applied
   * @param maxAngle The maximum angle, which will be clamped between wristMinMinAngle and
   *     wristMaxMaxAngle before being applied
   */
  public void setAllowedRangeOfMotion(Angle minAngle, Angle maxAngle) {
    setMinAngle(minAngle);
    setMaxAngle(maxAngle);
  }

  /**
   * Sets the minimum allowed angle that the wrist may target.
   *
   * <p>When not in override mode, the goal angle of the wrist will be clamped to be between these
   * values before it is sent to the IO. When these clamps change, the original goal angle is
   * clamped to be within the new bounds.
   *
   * @param minAngle The minimum angle, which will be clamped between wristMinMinAngle and
   *     wristMaxMaxAngle before being applied
   */
  public void setMinAngle(Angle minAngle) {
    this.minAngle.mut_replace(
        UnitUtils.clampMeasure(
            minAngle,
            JsonConstants.wristConstants.wristMinMinAngle,
            JsonConstants.wristConstants.wristMaxMaxAngle));

    Logger.recordOutput("wrist/minAngle", minAngle);
  }

  /**
   * Sets the maximum allowed angle that the wrist may target.
   *
   * <p>When not in override mode, the goal angle of the wrist will be clamped to be between these
   * values before it is sent to the IO. When these clamps change, the original goal angle is
   * clamped to be within the new bounds.
   *
   * @param maxAngle The maximum angle, which will be clamped between wristMinMinAngle and
   *     wristMaxMaxAngle before being applied
   */
  public void setMaxAngle(Angle maxAngle) {
    this.maxAngle.mut_replace(
        UnitUtils.clampMeasure(
            maxAngle,
            JsonConstants.wristConstants.wristMaxMaxAngle,
            JsonConstants.wristConstants.wristMaxMaxAngle));

    Logger.recordOutput("wrist/maxAngle", maxAngle);
  }

  /**
   * Get the current angle of the wrist
   *
   * @return
   */
  public Angle getWristAngle() {
    return inputs.wristPosition;
  }

  /**
   * Get a reference to the wrist's IO. This should be used to update PID, motion profile, and feed
   * forward gains, and to set brake mode/disable motors. This method exists to avoid the need to
   * duplicate all of these functions between the mechanism and the IO.
   *
   * @return the wrist mechanism's IO
   */
  public WristIO getIO() {
    return io;
  }

  /** Set whether or not the motor on the wrist should be disabled */
  public void setMotorsDisabled(boolean disabled) {
    io.setMotorsDisabled(disabled);
  }
}
