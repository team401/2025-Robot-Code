package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ElevatorIO.ElevatorOutputMode;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import org.littletonrobotics.junction.Logger;

public class IntakeState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  // Keep track of these so we can update it in tuning modes
  private static Angle intakeAnglePastCoralRange =
      JsonConstants.clawConstants.intakeAnglePastCoralrange;
  private static Angle intakeAnglePastAlgaeRange =
      JsonConstants.clawConstants.intakeAnglePastAlgaerange;

  /**
   * Keep track of what angle the motor was at when the sensor first detected the game piece.
   *
   * <p>The thinking here is that it will operate like a stopwatch: store the initial angle on
   * detect and then get the current angle to see how much it's rotated since then. r
   */
  private MutAngle detectedAngle = Rotations.mutable(0.0);

  /** Are we currently counting rotations to stop intake */
  private boolean isCounting;

  /** Track if the elevator has moved in the intake state. */
  private boolean hasMovedYet = false;

  private MedianFilter velocityFilter =
      new MedianFilter(JsonConstants.elevatorConstants.homingVelocityFilterWindowSize);

  private Timer homingTimer = new Timer();

  public IntakeState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  public void onEntry(Transition transition) {
    detectedAngle = Rotations.mutable(0.0);
    isCounting = false;
    Logger.recordOutput("claw/intake/diff", 0.0);

    // Reset variables to do homing.
    hasMovedYet = false;
    velocityFilter.reset();
    homingTimer.reset();
    homingTimer.start();
  }

  // @Override
  public void periodic() {
    if (!DriverStation.isEnabled()) {
      homingTimer.restart();
      return;
    }

    if (scoringSubsystem.getGamePiece() == GamePiece.Algae) {
      scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.algaeIntakeVoltage);
    } else {
      scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.coralIntakeVoltage);
    }

    ScoringSetpoint setpoint;
    switch (scoringSubsystem.getGamePiece()) {
      case Coral:
        setpoint = JsonConstants.scoringSetpoints.coralStation;
        break;
      case Algae:
        switch (scoringSubsystem.getAlgaeIntakeTarget()) {
          case L2:
            setpoint = JsonConstants.scoringSetpoints.L2algae;
            break;
          case Ground:
            setpoint = JsonConstants.scoringSetpoints.ground;
            break;
          default:
            // TODO: Decide if we want to default to L3 or ground
            System.out.println(
                "ERROR: Scoring commanded to intake algae but FieldTarget was "
                    + scoringSubsystem.getCoralTarget().toString()
                    + " (not L2, L3, or Ground). Defaulted to L3.");
          case L3:
            setpoint = JsonConstants.scoringSetpoints.L3algae;
            break;
        }
        break;
      default:
        System.out.println(
            "ERROR: Unkown GamePiece "
                + scoringSubsystem.getGamePiece()
                + " (defaulted to idle setpoint)");
        setpoint = JsonConstants.scoringSetpoints.idle;
        break;
    }

    scoringSubsystem.setGoalSetpoint(setpoint);

    // Seed the elevator to 0 when at the bottom of intake when intaking a coral.
    if (scoringSubsystem.getGamePiece() == GamePiece.Coral) {
      scoringSubsystem.setElevatorOverrideVoltage(JsonConstants.elevatorConstants.homingVoltage);
      scoringSubsystem.setElevatorOverrideMode(ElevatorOutputMode.Voltage);

      double filteredAbsVelocity =
          velocityFilter.calculate(scoringSubsystem.getElevatorVelocity().abs(MetersPerSecond));
      if (filteredAbsVelocity
          < JsonConstants.elevatorConstants.homingVelocityThresholdMetersPerSecond) {
        // If the elevator is NOT moving:
        if (hasMovedYet) {
          // If it HAS moved yet, that means it's moved and then come to rest, therefore we are at
          // zero
          System.out.println("Homed by moving then stopping");
          scoringSubsystem.seedElevatorToZero();
        } else if (homingTimer.hasElapsed(
            JsonConstants.elevatorConstants.homingMaxUnmovingTime.in(Seconds))) {
          // If it hasn't moved yet, and it's been longer than our threshold, we're satisfied that
          // it
          // was already at zero
          System.out.println("Homed by never moving");
          scoringSubsystem.seedElevatorToZero();
        }
      } else {
        // If the elevator IS moving, we keep track of that in hasMovedYet so we'll know when it
        // stops
        hasMovedYet = true;
      }
    }

    switch (scoringSubsystem.getGamePiece()) {
      case Coral:
        if (scoringSubsystem.isCoralDetected()) {
          scoringSubsystem.setClawRollerVoltage(Volts.zero());
          scoringSubsystem.fireTrigger(ScoringTrigger.DoneIntaking);
        }
        break;
      case Algae:
        boolean algaeCurrentDetected = scoringSubsystem.isAlgaeCurrentDetected();

        if (scoringSubsystem.isAlgaeDetected() || algaeCurrentDetected) {
          scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.algaeIdleVoltage);
          scoringSubsystem.fireTrigger(ScoringTrigger.DoneIntaking);

          if (algaeCurrentDetected) {
            scoringSubsystem.setAlgaeCurrentDetected(true);
          }
        }
        break;
    }

    Logger.recordOutput("claw/intake/detectedAngle", detectedAngle);
  }

  public static void setIntakeAnglePastCoralrange(Angle newAngle) {
    intakeAnglePastCoralRange = newAngle;
  }

  public static void setIntakeAnglePastAlgaerange(Angle newAngle) {
    intakeAnglePastAlgaeRange = newAngle;
  }

  @Override
  public void onExit(Transition transition) {
    scoringSubsystem.setElevatorOverrideMode(ElevatorOutputMode.ClosedLoop);
  }
}
