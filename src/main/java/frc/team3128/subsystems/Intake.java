package frc.team3128.subsystems;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3128.Constants.IntakeConstants.*;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

/**
 * Class for the Intake Subsystem 
 */

public class Intake extends SubsystemBase {

    private static Intake instance;

    private DoubleSolenoid m_intakeSolenoid;
    private NAR_TalonSRX m_intake;

    public Intake() {
        configMotors();
        configPneumatics();
    }

    public static synchronized Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    /**
     * Initializes motors and sets up CAN frame periods
     */
    private void configMotors () {
        m_intake = new NAR_TalonSRX(INTAKE_MOTOR_ID);
        m_intake.setInverted(true);

        // set CAN status frame periods
        m_intake.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 45);
        m_intake.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 45);
        m_intake.setControlFramePeriod(ControlFrame.Control_3_General, 20);
    }

    /**
     * Initializes pneumatics 
     */
    private void configPneumatics() {
        m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                                INTAKE_SOLENOID_FORWARD_CHANNEL_ID, 
                                                INTAKE_SOLENOID_BACKWARD_CHANNEL_ID);
        retractIntake();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", m_intake.get());
        SmartDashboard.putString("Intake State", getSolenoid());
    }

    /**
     * Runs intake motor forward at default power 
     */
    public void runIntake(){
        m_intake.set(ControlMode.PercentOutput, INTAKE_MOTOR_POWER);
    }

    /**
     * Runs intake motor forward at parameter power 
     */
    public void runIntake(double power) {
        m_intake.set(ControlMode.PercentOutput, power);
    }

    /**
     * Stops intake motor
     */
    public void stopIntake(){
        m_intake.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Retracts intake pneumatics to upright state
     */
    public void retractIntake(){
        m_intakeSolenoid.set(kReverse);
    }

    /**
     * Ejects intake pneumatics to out/ejected state
     */
    public void ejectIntake(){
        m_intakeSolenoid.set(kForward);
    }

    /**
     * Gets current intake pneumatics state
     */
    public String getSolenoid() {
        return m_intakeSolenoid.get().toString();
    }
    
}

