package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import coppercore.wpilib_interface.Controllers;
import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.StrategyManager.AutonomyMode;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbAction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.drive.ReefLineupUtil;
import frc.robot.subsystems.ramp.RampSubsystem;
import frc.robot.subsystems.ramp.states.RampState.RampTriggers;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import java.util.List;
import java.util.function.DoubleSupplier;

public final class InitBindings {

  private static List<Controllers.Controller> controllers;

  public static void initControllers() {
    OperatorConstants.synced.loadData();
    Controllers.synced.setFile(
        EnvironmentHandler.getEnvironmentHandler()
            .getEnvironmentPathProvider()
            .resolveReadPath(OperatorConstants.synced.getObject().mappingFile));
    Controllers.loadControllers();

    controllers = Controllers.getControllers();
    manualScore = getButton("ManualScore");
    warmupPressed = getButton("OTF");
    isIntakeHeld = getButton("Intake");
  }

  public static DoubleSupplier getAxis(String command) {
    for (Controllers.Controller controller : controllers) {
      if (controller.hasAxis(command)) {
        return controller.getAxis(command);
      }
    }
    System.out.println("Could not find Axis with command: " + command);
    return () -> 0;
  }

  public static Trigger getButton(String command) {
    for (Controllers.Controller controller : controllers) {
      if (controller.hasButton(command)) {
        return controller.getButton(command);
      }
    }
    System.out.println("Could not find Button with command: " + command);
    return new Trigger(() -> false);
  }

  public static void initDriveBindings(Drive drive, StrategyManager strategyManager) {
    StrategyManager.driveCommand =
        new DriveWithJoysticks(
            drive,
            () -> getAxis("driveX").getAsDouble(),
            () -> getAxis("driveY").getAsDouble(),
            () -> getAxis("driveRotation").getAsDouble(),
            JsonConstants.drivetrainConstants.maxLinearSpeedComp,
            JsonConstants.drivetrainConstants.maxAngularSpeedComp,
            JsonConstants.drivetrainConstants.joystickDeadband);

    // hold right joystick trigger down to have drive go to desired location
    getButton("OTF")
        .onTrue(
            new InstantCommand(
                () -> {
                  switch (strategyManager.getAutonomyMode()) {
                    case Full:
                      // If a binding is used in full autonomy, switch to smart autonomy
                      strategyManager.setAutonomyMode(AutonomyMode.Smart);
                      // And then fall through to the smart autonomy behavior (no break here is
                      // intentional)
                    case Smart:
                      if (ScoringSubsystem.getInstance() != null) {
                        // Update scoring locations (drive and scoring subsystems) from snakescreen
                        // This is required here because it doesn't happen in periodic in Smart mode
                        // to avoid undoing the auto algae height selector
                        // The drive location is immediately overwritten below
                        strategyManager.updateScoringLocationsFromSnakeScreen();
                        strategyManager.updateScoringLevelFromNetworkTables();
                      }

                      // When the scoring trigger is pulled in smart autonomy, select the closest
                      // reef pole to score on if coral
                      if (ScoringSubsystem.getInstance() == null
                          || ScoringSubsystem.getInstance().getGamePiece() == GamePiece.Coral) {
                        drive.setDesiredLocation(
                            ReefLineupUtil.getClosestReefLocation(drive.getPose()));
                      }
                      // Then fall through to scheduling OTF like in mixed autonomy (no break here
                      // is intentional)
                    case Mixed:
                      if (ScoringSubsystem.getInstance() == null
                          || ScoringSubsystem.getInstance().getGamePiece() == GamePiece.Coral) {
                        drive.setGoToIntake(false);
                        drive.fireTrigger(DriveTrigger.BeginLinear);
                      } else if (ScoringSubsystem.getInstance() != null) {
                        ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.StartWarmup);
                      }
                      break;
                    case Manual:
                      // Only start scoring warmup if in manual autonomy; in mixed and full,
                      // drivetrain triggers this when it enters lineup
                      if (ScoringSubsystem.getInstance() != null) {
                        ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.StartWarmup);
                      }
                      break;
                  }
                },
                drive));

    getButton("OTF")
        .onFalse(
            new InstantCommand(
                () -> {
                  switch (strategyManager.getAutonomyMode()) {
                    case Full:
                      // If a binding is used in full autonomy, switch to mixed autonomy
                      strategyManager.setAutonomyMode(AutonomyMode.Smart);
                      // And then fall through to the smart/mixed autonomy behavior (no break here
                      // is
                      // intentional)
                    case Smart:
                    case Mixed:
                      // Cancel auto align if in mixed autonomy
                      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
                      // Then always cancel warmup for scoring (no break here is intentional)
                    case Manual:
                      if (ScoringSubsystem.getInstance() != null) {
                        ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.CancelWarmup);
                      }
                      break;
                  }
                },
                drive));

    getButton("seed_rotation")
        .onTrue(
            new InstantCommand(
                () -> {
                  // Left joystick top button seeds direction as forward
                  drive.seedDirectionForward();
                },
                drive));

    getButton("sidestep")
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.sidestepReefLocation();
                }));

    // // pov right (reef 0-11 -> processor left -> processor right )
    // // pov left (goes backwards of right)
    // driverController
    //     .povRight()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               DesiredLocationSelector.incrementIndex();
    //               DesiredLocationSelector.setLocationFromIndex(drive);
    //             }));
    // driverController
    //     .povLeft()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               DesiredLocationSelector.decrementIndex();
    //               DesiredLocationSelector.setLocationFromIndex(drive);
    //             },
    //             drive));s

    // TODO: Rebind localization if necessary
    // leftJoystick
    //     .top()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               drive.setPose(
    //                   new Pose2d(
    //                       Meters.of(14.350), Meters.of(4.0), new Rotation2d(Degrees.of(180))));
    //             }));

    // Hold left trigger to intake, release to cancel
    getButton("intake")
        .onTrue(
            new InstantCommand(
                () -> {
                  switch (strategyManager.getAutonomyMode()) {
                    case Full:
                      // If a binding is used in full autonomy, switch to mixed autonomy
                      strategyManager.setAutonomyMode(AutonomyMode.Mixed);
                      // And then fall through to the mixed autonomy behavior (no break here is
                      // intentional)
                    case Smart:
                      // If in algae mode, automatically set level and pick nearest algae location
                      if (ScoringSubsystem.getInstance() == null
                          || ScoringSubsystem.getInstance().getGamePiece() == GamePiece.Algae) {
                        DesiredLocation desiredLocation =
                            ReefLineupUtil.getClosestAlgaeLocation(drive.getPose());
                        // drive.setDesiredIntakeLocation(desiredLocation);

                        // Set algae level automatically
                        if (ScoringSubsystem.getInstance() != null) {
                          ScoringSubsystem.getInstance()
                              .setTarget(
                                  ReefLineupUtil.getAlgaeLevelFromDesiredLocation(desiredLocation));
                        }
                      } else {
                        if (DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red) {
                          drive.setDesiredIntakeLocation(
                              JsonConstants.redFieldLocations.getClosestCoralStation(
                                  drive.getPose()));
                        } else {
                          drive.setDesiredIntakeLocation(
                              JsonConstants.blueFieldLocations.getClosestCoralStation(
                                  drive.getPose()));
                        }
                      }
                    case Mixed:
                      // Start auto align if in mixed autonomy
                      if (ScoringSubsystem.getInstance() != null
                          && ScoringSubsystem.getInstance().getGamePiece() == GamePiece.Coral
                          && DriverStation.isAutonomous()) {
                        drive.setGoToIntake(true);
                        drive.fireTrigger(DriveTrigger.BeginOTF);
                      }
                      // Then always start intake for scoring (no break here is intentional)
                    case Manual:
                      if (ScoringSubsystem.getInstance() != null) {
                        ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.BeginIntake);
                      }
                      break;
                  }
                },
                drive));
    getButton("intake")
        .onFalse(
            new InstantCommand(
                () -> {
                  switch (strategyManager.getAutonomyMode()) {
                    case Full:
                      // If a binding is used in full autonomy, switch to mixed autonomy
                      strategyManager.setAutonomyMode(AutonomyMode.Mixed);
                      // And then fall through to the mixed autonomy behavior (no break here is
                      // intentional)
                    case Smart:
                    case Mixed:
                    case Manual:
                      // Cancel auto align if in mixed autonomy
                      drive.setGoToIntake(false);
                      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
                      // Then always cancel intake for scoring (no break here is intentional)
                      if (ScoringSubsystem.getInstance() != null) {
                        ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.CancelIntake);
                      }
                      break;
                  }
                },
                drive));
    drive.setDefaultCommand(StrategyManager.driveCommand);
  }

  public static void initRampBindings(RampSubsystem rampSubsystem) {
    // driverController
    //     .a()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               rampSubsystem.fireTrigger(RampTriggers.START_CLIMB);
    //             }));
    // driverController
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               rampSubsystem.fireTrigger(RampTriggers.START_INTAKE);
    //             }));

    getButton("OTF")
        .onTrue(
            new InstantCommand(
                () -> {
                  rampSubsystem.fireTrigger(RampTriggers.RETURN_TO_IDLE);
                }));

    getButton("OTF")
        .onFalse(
            new InstantCommand(
                () -> {
                  rampSubsystem.fireTrigger(RampTriggers.RETURN_TO_IDLE);
                }));

    getButton("intake")
        .onTrue(
            new InstantCommand(
                () -> {
                  rampSubsystem.fireTrigger(RampTriggers.INTAKE);
                }));

    getButton("intake")
        .onFalse(
            new InstantCommand(
                () -> {
                  rampSubsystem.fireTrigger(RampTriggers.RETURN_TO_IDLE);
                }));

    getButton("climb")
        .onTrue(
            new InstantCommand(
                () -> {
                  rampSubsystem.fireTrigger(RampTriggers.CLIMB);
                }));

    getButton("cancel_climb")
        .onTrue(
            new InstantCommand(
                () -> {
                  // rampSubsystem.fireTrigger(RampTriggers.RETURN_TO_IDLE);
                }));
    getButton("home_ramp")
        .onTrue(
            new InstantCommand(
                () -> {
                  rampSubsystem.fireTrigger(RampTriggers.HOME);
                }));
  }

  public static void initClimbBindings(ClimbSubsystem climb) {
    // driverController.a().onTrue(new InstantCommand(() -> climb.fireTrigger(ClimbAction.CLIMB)));
    // driverController.b().onTrue(new InstantCommand(() -> climb.fireTrigger(ClimbAction.CANCEL)));

    // TODO: Find actual numbers for these buttons using driverstation
    getButton("climb").onTrue(new InstantCommand(() -> climb.fireTrigger(ClimbAction.CLIMB)));

    getButton("cancel_climb")
        .onTrue(
            new InstantCommand(
                () -> {
                  climb.fireTrigger(ClimbAction.CANCEL);
                }));
  }

  public static void initScoringBindings(ScoringSubsystem scoring) {
    getButton("SecondIntake")
        .onTrue(
            new InstantCommand(
                () -> {
                  scoring.fireTrigger(ScoringTrigger.BeginIntake);
                }));
    getButton("L4Score")
        .onTrue(
            new InstantCommand(
                () -> {
                  scoring.setTarget(FieldTarget.L4);
                  scoring.setGamePiece(GamePiece.Coral);
                  scoring.fireTrigger(ScoringTrigger.StartWarmup);
                }));
    getButton("RunClawCoralScore")
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

    getButton("ManualScore")
        .onTrue(
            new InstantCommand(
                () -> {
                  if (ScoringSubsystem.getInstance().getGamePiece() == GamePiece.Algae
                      && ScoringSubsystem.getInstance().getAlgaeScoreTarget()
                          == FieldTarget.Processor) {
                    ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.WarmupReady);
                  } else if (ScoringSubsystem.getInstance().getGamePiece() == GamePiece.Coral) {
                    ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.WarmupReady);
                  }
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
        getButton("SetpointTuningClawForward")
            .onTrue(
                new InstantCommand(
                    () -> {
                      ScoringSubsystem.getInstance()
                          .setClawRollerVoltage(JsonConstants.clawConstants.coralIntakeVoltage);
                    }))
            .onFalse(
                new InstantCommand(
                    () -> {
                      ScoringSubsystem.getInstance().setClawRollerVoltage(Volts.zero());
                    }));

        // Use A to manually run claw backward (good for repositioning coral)
        getButton("SetpointTuningClawBackward")
            .onTrue(
                new InstantCommand(
                    () -> {
                      ScoringSubsystem.getInstance()
                          .setClawRollerVoltage(
                              JsonConstants.clawConstants.coralIntakeVoltage.times(-0.5));
                    }))
            .onFalse(
                new InstantCommand(
                    () -> {
                      ScoringSubsystem.getInstance().setClawRollerVoltage(Volts.zero());
                    }));

        // Use Y to manually score coral
        getButton("SetpointTuningScoreCoral")
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
            .setTuningHeightSetpointAdjustmentSupplier(getAxis("SetpointTuningElevator"));
      default:
        break;
    }
  }

  private static Trigger manualScore;
  private static Trigger warmupPressed;
  private static Trigger isIntakeHeld;

  public static boolean isManualScorePressed() {
    return manualScore.getAsBoolean();
  }

  public static boolean isWarmupPressed() {
    return warmupPressed.getAsBoolean();
  }

  public static boolean isIntakeHeld() {
    return isIntakeHeld.getAsBoolean();
  }
}
