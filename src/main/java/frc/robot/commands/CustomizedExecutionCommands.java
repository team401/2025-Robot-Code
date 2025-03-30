package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;


public class CustomizedExecutionCommands {
    

    private static class ConditionalCommandExecution extends WrapperCommand {
        
        protected BooleanSupplier condition;

        public ConditionalCommandExecution(Command command, BooleanSupplier condition){
            super(command);
            this.condition = condition;
        }

        @Override
        public void execute(){
            if (condition.getAsBoolean()){
                m_command.execute();
            }
        }
    }

    private static class InitializedConditionalStartCommand extends WrapperCommand {
        
        protected BooleanSupplier condition;
        protected boolean canRun;

        public InitializedConditionalStartCommand(Command command, BooleanSupplier condition){
            super(command);
            this.condition = condition;
        }

        @Override
        public void execute(){
            canRun = canRun || condition.getAsBoolean();
            if (canRun){
                m_command.execute();
            }
        }
    }

    public static Command executeIf(Command command, BooleanSupplier condition){
        return new ConditionalCommandExecution(command, condition);
    }

    public static Command initilizedOnlyIf(Command command, BooleanSupplier condition){
        return new InitializedConditionalStartCommand(command, condition);
    }
}
