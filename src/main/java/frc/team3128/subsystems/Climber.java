package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3128.hardware.NAR_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Climber implements Subsystem {
    
    private NAR_TalonFX CLIMBER_MOTOR_1, CLIMBER_MOTOR_2;
    public static final Climber instance = new Climber(); 

    public Climber() {
        configMotors(); 
    }

    private void configMotors() {
        CLIMBER_MOTOR_1 = new NAR_TalonFX(Constants.ClimberConstants.CLIMBER_MOTOR_1_ID, true);
        CLIMBER_MOTOR_2 = new NAR_TalonFX(Constants.ClimberConstants.CLIMBER_MOTOR_2_ID, true);
    }

    public void setMotors(double power) {
        CLIMBER_MOTOR_1.set(ControlMode.PercentOutput, power);
        CLIMBER_MOTOR_2.set(ControlMode.PercentOutput, power);
    }

}
