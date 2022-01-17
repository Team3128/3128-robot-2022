package frc.team3128.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants;
import frc.team3128.common.hardware.motor.NAR_TalonSRX;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


public class Intake extends SubsystemBase {


    public static final Intake instance = new Intake();
    private DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
    private NAR_TalonSRX m_intake;


    public static Intake getInstance(){
        return instance; 
    }

    public Intake() {
        configMotors();

    }

    private void configMotors () {
        m_intake = new NAR_TalonSRX(Constants.IntakeConstants.m_intake_ID);

    }

    public void runIntake(){
        m_intake.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_POWER);
    }

    public void stopIntake(){
        m_intake.set(ControlMode.PercentOutput, 0);
    }

    public void retractIntake(){
        m_solenoid.set(kReverse);
    }

    public void ejectIntake(){
        m_solenoid.set(kForward);

    }
    public void off() {
        m_solenoid.set(kOff); 
    }





}

