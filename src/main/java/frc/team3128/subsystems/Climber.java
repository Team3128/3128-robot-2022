package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants;
import frc.team3128.common.NAR_EMotor;
import frc.team3128.common.hardware.motor.NAR_TalonSRX;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Climber extends SubsystemBase {
    private static Climber instance;
    private DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
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
        m_climbMotor2.follow((NAR_EMotor)m_climbMotor1);
        m_climbMotor1.setNeutralMode(Constants.ClimberConstants.CLIMBER_NEUTRAL_MODE);
        m_solenoid.set(kOff);  
    }

    public void climberRetract(){
        m_climbMotor1.set(Constants.ClimberConstants.CLIMBER_POWER);
    }
    public void climberExtend(){
        m_climbMotor1.set(-Constants.ClimberConstants.CLIMBER_POWER);
    }
    public void angleArm(){
        m_solenoid.set(kForward);
    }
    public void unAngleArm(){
        m_solenoid.set(kReverse);
    }
    public void climbEnd(){
        m_solenoid.set(kOff);
        m_climbMotor1.set(0);
    }

}