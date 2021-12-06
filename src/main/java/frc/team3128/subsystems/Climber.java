package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motor.NAR_TalonSRX;

public class Climber extends SubsystemBase {

    private static Climber instance;
    
    private NAR_TalonSRX m_climbMotor1, m_climbMotor2;

    public Climber() {
        configMotors();
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    private void configMotors() {
        m_climbMotor1 = new NAR_TalonSRX(Constants.ClimberConstants.CLIMBER_MOTOR_1_ID);
        m_climbMotor2 = new NAR_TalonSRX(Constants.ClimberConstants.CLIMBER_MOTOR_2_ID);
        
        m_climbMotor1.setNeutralMode(Constants.ClimberConstants.CLIMBER_NEUTRAL_MODE);
        m_climbMotor2.setNeutralMode(Constants.ClimberConstants.CLIMBER_NEUTRAL_MODE);
    }
    
    public void moveClimberUp() {
        //If ControlMode is not specified, WPI_TalonSRX will default to PercentOutput
        m_climbMotor1.set(Constants.ClimberConstants.CLIMBER_UP_POWER);
        m_climbMotor2.set(Constants.ClimberConstants.CLIMBER_UP_POWER);
    }

    public void moveClimberDown() {
        m_climbMotor1.set(Constants.ClimberConstants.CLIMBER_DOWN_POWER);
        m_climbMotor2.set(Constants.ClimberConstants.CLIMBER_DOWN_POWER);
    }

    public void stopClimber() {
        m_climbMotor1.set(0);
        m_climbMotor2.set(0);
    }

}
