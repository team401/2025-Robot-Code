package frc.robot.subsystems.scoring.wrist2;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.UnitUtils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.scoring.wrist2.WristIO.WristOutputMode;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism to manage the Wrist
 *
 * <ul>
 *   <li>Uses closed-loop TorqueCurrentFOC control
 */
public class WristMechanism {
  WristIO io;
  WristInputsAutoLogged inputs = new WristInputsAutoLogged();
  WristOutputsAutoLogged outputs = new WristOutputsAutoLogged();

  MutAngle goalAngle = Rotations.mutable(0.0);
  MutAngle clampedGoalAngle = Rotations.mutable(0.0);

  MutAngle minAngle = WristConstants.synced.getObject().wristMinMinAngle.mutableCopy();
  MutAngle maxAngle = WristConstants.synced.getObject().wristMaxMaxAngle.mutableCopy();

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
  LoggedTunableNumber wristTuningOverrideVolts;

  public WristMechanism(WristIO io) {
    wristkP =
        new LoggedTunableNumber("WristTunables/wristkP", WristConstants.synced.getObject().wristKP);
    wristkI =
        new LoggedTunableNumber("WristTunables/wristkI", WristConstants.synced.getObject().wristKI);
    wristkD =
        new LoggedTunableNumber("WristTunables/wristkD", WristConstants.synced.getObject().wristKD);

    wristkS =
        new LoggedTunableNumber("WristTunables/wristkS", WristConstants.synced.getObject().wristKS);
    wristkV =
        new LoggedTunableNumber("WristTunables/wristkV", WristConstants.synced.getObject().wristKV);
    wristkA =
        new LoggedTunableNumber("WristTunables/wristkA", WristConstants.synced.getObject().wristKA);
    wristkG =
        new LoggedTunableNumber("WristTunables/wristkG", WristConstants.synced.getObject().wristKG);

    wristCruiseVelocity =
        new LoggedTunableNumber(
            "WristTunables/wristCruiseVelocity",
            WristConstants.synced.getObject().wristAngularCruiseVelocityRotationsPerSecond);
    wristExpokV =
        new LoggedTunableNumber(
            "WristTunables/wristExpokV", WristConstants.synced.getObject().wristMotionMagicExpo_kV);
    wristExpokA =
        new LoggedTunableNumber(
            "WristTunables/wristExpokA", WristConstants.synced.getObject().wristMotionMagicExpo_kA);

    wristTuningSetpointRotations =
        new LoggedTunableNumber("WristTunables/wristTuningSetpointRotations", 0.0);
    wristTuningOverrideVolts =
        new LoggedTunableNumber("WristTunables/wristTuningOverrideVolts", 0.0);

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

    Logger.processInputs("Wrist/inputs", inputs);
    Logger.processInputs("Wrist/outputs", outputs);
  }

  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }

  /** This method must be called from the subsystem's test periodic! */
  public void testPeriodic() {
    if (false) { // TODO: Replace placeholder test if WristTuning mode is active
      // switch (TestModeManager.getTestMode()) {
      // case WristClosedLoopTuning:
        io.setOutputMode(WristOutputMode.ClosedLoop);
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
      /*  case WristVoltageTuning:
          LoggedTunableNumber.ifChanged(
            hashCode(),
            (setpoint) -> {
              io.setOverrideVoltage(Volts.of(setpoint[0]));
            },
            wristTuningOverrideVolts);
          io.setOverrideMode(true);
          break;
        }
        */
    }
  }

  public void sendGoalAngleToIO() {
    updateClampedGoalAngle();
    io.setWristEncoderGoalPos(clampedGoalAngle);
  }

  /**
   * Based on the bounds previously set, clamp the last set goal angle to be between the bounds.
   *
   * <p>If the goal height is outside of the bounds and the bounds are expanded, this function will
   * still behave as expected, as the mechanism remembers its unclamped goal height and will attempt
   * to get there once it is allowed.
   */
  private void updateClampedGoalAngle() {
    clampedGoalAngle.mut_replace(UnitUtils.clampMeasure(goalAngle, minAngle, maxAngle));

    Logger.recordOutput("Wrist/clampedGoalAngle", clampedGoalAngle);
  }

  /**
   * Set the goal angle the wrist will to control to.
   *
   * <p>This goal angle will be clamped by the allowed range of motion
   *
   * @param goalAngle The new goal angle
   */
  public void setGoalAngle(Angle goalAngle) {
    this.goalAngle.mut_replace(goalAngle);

    Logger.recordOutput("Wrist/goalAngle", goalAngle);
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
            WristConstants.synced.getObject().wristMinMinAngle,
            WristConstants.synced.getObject().wristMaxMaxAngle));

    Logger.recordOutput("Wrist/minAngle", minAngle);
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
            WristConstants.synced.getObject().wristMaxMaxAngle,
            WristConstants.synced.getObject().wristMaxMaxAngle));

    Logger.recordOutput("Wrist/maxAngle", maxAngle);
  }

  /**
   * Get the current angle of the wrist
   *
   * @return
   */
  public Angle getWristAngle() {
    return inputs.wristEncoderPos;
  }

  /**
   * Get the current velocity of the wrist
   *
   * @return The current velocity of the wrist, according to the wristEncoder
   */
  public AngularVelocity getWristVelocity() {
    return inputs.wristEncoderVel;
  }

  /**
   * Check whether or not the wristEncoder is currently connected.
   *
   * <p>"Connected" means that last time the position and velocity status signals were refreshed, the status code
   * was OK
   *
   * @return True if connected, false if disconnected
   */
  public boolean isWristEncoderConnected() {
    return inputs.wristEncoderConnected;
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

  /** Get the current unclamped goal angle of the wrist */
  public Angle getGoalAngle() {
    return goalAngle;
  }
}