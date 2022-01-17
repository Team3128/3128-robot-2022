package frc.team3128.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants;
import frc.team3128.common.hardware.motor.NAR_TalonSRX;


public class Intake extends SubsystemBase {


    public static final Intake instance = new Intake();
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





}

