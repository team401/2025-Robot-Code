package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

import frc.robot.utils.monitors.Monitor;
import frc.robot.utils.monitors.MonitoredSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class BatterySubsystem extends MonitoredSubsystem {

        /*addMonitor(
                new Monitor(
                        "armEncoderUnplugged", // Name to log callback under
                        true, // Sticky fault (remain faulted even after conditions become
                        // acceptable again
                        () ->
                                // isStateValid function to check whether the state is currently
                                // valid or
                                // not
                                !(DriverStation.isEnabled()
                                        && (aimerInputs.aimAppliedVolts
                                                        > ScoringConstants.aimerkS * 2
                                                && aimerInputs.aimVelocityRadPerSec
                                                        < ScoringConstants
                                                                .aimerMovementThresholdRadPerSec)),
                        ScoringConstants
                                .maxAimUnresponsiveTimeSeconds, // timeToFault (how long can state
                        // be invalid before a
                        // fault occurs)
                        () -> { // faultCallback, the function run every tick when the monitor has
                            // detected a
                            // fault.
                            aimerIo.setOverrideMode(true);
                            aimerIo.setOverrideVolts(0.0);
                        })));
    */
}