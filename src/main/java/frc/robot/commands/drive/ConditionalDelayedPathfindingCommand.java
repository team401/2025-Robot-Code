package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CustomizedExecutionCommands;

public class ConditionalDelayedPathfindingCommand {
    
    public static Command generateCommand(Pose2d pose, PathConstraints constraints, double goalEndVelocity, BooleanSupplier conditionToStart){
        return generateCommand(AutoBuilder.pathfindToPose(pose, constraints, goalEndVelocity), conditionToStart);
    }

    public static Command generateCommand(Command pathFindingCommand, BooleanSupplier conditionToStart){
        return CustomizedExecutionCommands.initilizedOnlyIf(pathFindingCommand, conditionToStart);
    }

}
