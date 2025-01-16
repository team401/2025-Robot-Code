package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbIOTalonFX implements ClimbIO {
    
    private TalonFX motor;
    private CANcoder angleSensor;
    

    public ClimbIOTalonFX() {
        motor = new TalonFX(0);

        angleSensor = new CANcoder(1);
    }

}
