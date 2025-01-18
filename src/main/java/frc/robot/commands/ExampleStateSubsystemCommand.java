package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.stateMachineExample.StateMachineExample;

public class ExampleStateSubsystemCommand extends Command {
    public StateMachineExample stateMachineSubsystem;
    public CommandType commandType;

    private static enum CommandType {
        OVERRIDE,
        IDLE,
        ACTION
    }

    private ExampleStateSubsystemCommand(StateMachineExample subsystem, CommandType type) {
        stateMachineSubsystem = subsystem;
        commandType = type;
    }

    public static ExampleStateSubsystemCommand Override(StateMachineExample subsystem) {
        return new ExampleStateSubsystemCommand(subsystem, CommandType.OVERRIDE);
    }

    public static ExampleStateSubsystemCommand Idle(StateMachineExample subsystem) {
        return new ExampleStateSubsystemCommand(subsystem, CommandType.IDLE);
    }

    public static ExampleStateSubsystemCommand Action(StateMachineExample subsystem) {
        return new ExampleStateSubsystemCommand(subsystem, CommandType.ACTION);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch (commandType) {
            case OVERRIDE:
                stateMachineSubsystem.setAction(StateMachineExample.ExampleActions.OVERRIDE);
                break;

            case IDLE:
                stateMachineSubsystem.setAction(StateMachineExample.ExampleActions.IDLE);
                break;

            case ACTION:
                stateMachineSubsystem.setAction(StateMachineExample.ExampleActions.ACTION);
                break;

            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
