package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.StrategyManager.AutonomyMode;
import frc.robot.commands.drive.DesiredLocationSelector;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbAction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.ramp.RampSubsystem;
import frc.robot.subsystems.ramp.states.RampState.RampTriggers;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

public final class InitBindings {
  // Controller
  private static final CommandJoystick leftJoystick =
      new CommandJoystick(OperatorConstants.synced.getObject().kLeftJoystickPort);

  private static final CommandJoystick rightJoystick =
      new CommandJoystick(OperatorConstants.synced.getObject().kRightJoystickPort);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.synced.getObject().kDriverControllerPort);

  public static void initDriveBindings(Drive drive, StrategyManager strategyManager) {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive, // type: DriveTemplate
            leftJoystick, // type: CommandJoystick
            rightJoystick, // type: CommandJoystick
            JsonConstants.drivetrainConstants.maxLinearSpeed, // type: double (m/s)
            JsonConstants.drivetrainConstants.maxAngularSpeed, // type: double (rad/s)
            JsonConstants.drivetrainConstants.joystickDeadband // type: double
            ));

    // hold right joystick trigger down to have drive go to desired location
    rightJoystick
        .button(1)
        .onTrue(
            new InstantCommand(
                () -> {
                  if (strategyManager.getAutonomyMode() != AutonomyMode.Manual) {
                    drive.fireTrigger(DriveTrigger.BeginAutoAlignment);
                  }
                  ;
                },
                drive));
    rightJoystick
        .button(1)
        .onFalse(
            new InstantCommand(
                () -> {
                  if (strategyManager.getAutonomyMode() != AutonomyMode.Manual) {
                    drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
                    ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.CancelWarmup);
                  }
                  ;
                },
                drive));

    leftJoystick
        .button(1)
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.angleController.reset(drive.getRotation().getRadians());
                  drive.alignToFieldElement();
                },
                drive));
    leftJoystick
        .button(1)
        .onFalse(
            new InstantCommand(
                () -> {
                  drive.disableAlign();
                },
                drive)); // pov right (reef 0-11 -> processor left -> processor right )
    // pov left (goes backwards of right)
    driverController
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> {
                  DesiredLocationSelector.incrementIndex();
                  DesiredLocationSelector.setLocationFromIndex(drive);
                }));
    driverController
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  DesiredLocationSelector.decrementIndex();
                  DesiredLocationSelector.setLocationFromIndex(drive);
                },
                drive));

    leftJoystick
        .top()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.setPose(
                      new Pose2d(
                          Meters.of(14.350), Meters.of(4.0), new Rotation2d(Degrees.of(180))));
                }));
    rightJoystick
        .top()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.setDesiredIntakeLocation(DesiredLocation.CoralStationRight);
                  drive.setGoToIntake(true);
                  drive.fireTrigger(DriveTrigger.BeginAutoAlignment);
                },
                drive));

    rightJoystick
        .top()
        .onFalse(
            new InstantCommand(
                () -> {
                  drive.setGoToIntake(false);
                  drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
                },
                drive));
  }

  public static void initRampBindings(RampSubsystem rampSubsystem) {
    driverController
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  rampSubsystem.fireTrigger(RampTriggers.START_CLIMB);
                }));
    driverController
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  rampSubsystem.fireTrigger(RampTriggers.START_INTAKE);
                }));
  }

  public static void initClimbBindings(ClimbSubsystem climb) {
    driverController.a().onTrue(new InstantCommand(() -> climb.fireTrigger(ClimbAction.CLIMB)));
    driverController.b().onTrue(new InstantCommand(() -> climb.fireTrigger(ClimbAction.CANCEL)));
  }

  public static void initScoringBindings(ScoringSubsystem scoring) {
    driverController
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  scoring.fireTrigger(ScoringTrigger.BeginIntake);
                }));
    driverController
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  scoring.setTarget(FieldTarget.L4);
                  scoring.setGamePiece(GamePiece.Coral);
                  scoring.fireTrigger(ScoringTrigger.StartWarmup);
                }));
    driverController
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  scoring.setClawRollerVoltage(JsonConstants.clawConstants.coralScoreVoltage);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  scoring.setClawRollerVoltage(Volts.zero());
                }));
  }

  /**
   * Initialize bindings specifically for Test Mode.
   *
   * <p>This function doesn't run automatically, and it should be called in testInit
   */
  public static void initTestModeBindings() {
    switch (TestModeManager.getTestMode()) {
      case SetpointTuning:
        // Use B to manually run claw forward
        driverController
            .b()
            .onTrue(
                new InstantCommand(
                    () -> {
                      ScoringSubsystem.getInstance()
                          .setClawRollerVoltage(JsonConstants.clawConstants.intakeVoltage);
                    }))
            .onFalse(
                new InstantCommand(
                    () -> {
                      ScoringSubsystem.getInstance().setClawRollerVoltage(Volts.zero());
                    }));

        // Use A to manually run claw backward (good for repositioning coral)
        driverController
            .a()
            .onTrue(
                new InstantCommand(
                    () -> {
                      ScoringSubsystem.getInstance()
                          .setClawRollerVoltage(
                              JsonConstants.clawConstants.intakeVoltage.times(-0.5));
                    }))
            .onFalse(
                new InstantCommand(
                    () -> {
                      ScoringSubsystem.getInstance().setClawRollerVoltage(Volts.zero());
                    }));

        // Use Y to manually score coral
        driverController
            .y()
            .onTrue(
                new InstantCommand(
                    () -> {
                      ScoringSubsystem.getInstance()
                          .setClawRollerVoltage(JsonConstants.clawConstants.coralScoreVoltage);
                    }))
            .onFalse(
                new InstantCommand(
                    () -> {
                      ScoringSubsystem.getInstance().setClawRollerVoltage(Volts.zero());
                    }));

        ScoringSubsystem.getInstance()
            .setTuningHeightSetpointAdjustmentSupplier(() -> driverController.getLeftY());
      default:
        break;
    }
  }
}
